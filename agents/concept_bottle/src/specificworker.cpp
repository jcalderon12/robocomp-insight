/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	//G->write_to_json_file("./"+agent_name+".json");
	delete viewer_3d;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
	GenericWorker::initialize();

	//dsr update signals
	//connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
	//connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
	//connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_node_attrs_slot);
	//connect(G.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &SpecificWorker::modify_edge_attrs_slot);
	//connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
	//connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

	bottle_dimension_display_timer = std::chrono::steady_clock::now();

	rt = G->get_rt_api();

	if (display_point_cloud)
	{
		points = std::make_shared<std::vector<std::tuple<float, float, float>>>();
		colors = std::make_shared<std::vector<std::tuple<float, float, float>>>();

		viewer_widget = new QWidget();
		viewer_widget->setWindowTitle("Bottle point cloud");
		viewer_widget->show();

		viewer_3d = new Viewer(viewer_widget, points, colors);
		viewer_3d->show();
	}
}

inline bool hasFallen(const SpecificWorker::BottlePose& pose, float threshold_degrees = 45.0f)
{
    float z_world = 1.0f - 2.0f * (pose.qx*pose.qx + pose.qy*pose.qy);
    float cos_threshold = std::cos(threshold_degrees * M_PI / 180.0f);
    return z_world < cos_threshold;
}


void SpecificWorker::compute()
{
	if(check_bottle_related_robot())
	{
		std::optional<SpecificWorker::BottlePose> bottle_pose = detect_bottle();

		if(!bottle_has_fallen(bottle_pose))
		{
			update_bottle_pose_in_dsr(bottle_pose);
		}
		else
		{
			// change_rt_from_robot_to_room(); // Here the agent should delete the RT, but for now we only change to 
			eliminate_rt_from_robot_to_bottle();
		}
	}
	else
	{
		std::optional<BottlePose> bottle_pose = detect_bottle();
		if (bottle_pose.has_value())
		{
			if bottle_pose->tilt_angle > tilt_threshold
				return;

			bottle_redetected_count++;
			if (bottle_redetected_count >= bottle_redetected_threshold)
			{
				bottle_redetected_count = 0; 
				std::cout << "Bottle related to robot again! Updating RT edge." << std::endl;
				update_bottle_pose_in_dsr(bottle_pose);
			}
		}
		else
			bottle_redetected_count = 0; 
	}
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}



//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

/**************************************/
/************ DSR METHODS *************/

bool SpecificWorker::check_bottle_related_robot()
{
	auto bottle_node_opt = G->get_node("bottle");
	if(!bottle_node_opt.has_value())
		return false;
	auto bottle_node = bottle_node_opt.value();

	auto robot_node_opt = G->get_node("robot");
	if(!robot_node_opt.has_value())
		return false;
	auto robot_node = robot_node_opt.value();

	auto edge_opt = G->get_edge(robot_node.id(), bottle_node.id(),"RT");
	if(!edge_opt.has_value())
		return false;

	return true;
}

SpecificWorker::EulerAngles  SpecificWorker::quaternion_to_euler(float qx, float qy, float qz, float qw)
{
    EulerAngles angles;

    // Roll (X)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y)
    double sinp = 2.0 * (qw * qy - qz * qx);

    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // 90°
    else
        angles.pitch = std::asin(sinp);

    // Yaw (Z)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

void SpecificWorker::update_bottle_pose_in_dsr(const std::optional<BottlePose>& bottle_pose)
{
	auto bottle_node_opt = G->get_node("bottle");
	if(!bottle_node_opt.has_value())
		return;
	auto bottle_node = bottle_node_opt.value();

	auto robot_node_opt = G->get_node("robot");
	if(!robot_node_opt.has_value())
		return;
	auto robot_node = robot_node_opt.value();

	if(bottle_pose.has_value())
	{
		auto pose = bottle_pose.value();
		
		// G->add_or_modify_attrib_local<obj_height_att>(bottle_node, (int)pose.height);
		// G->add_or_modify_attrib_local<obj_width_att>(bottle_node, (int)pose.width);
		// G->add_or_modify_attrib_local<obj_depth_att>(bottle_node, (int)pose.depth);

		// G->update_node(bottle_node);

		auto euler_angles = quaternion_to_euler(pose.qx, pose.qy, pose.qz, pose.qw);

		rt->insert_or_assign_edge_RT(robot_node, bottle_node.id(), std::vector<float>{pose.x, pose.y, pose.z}, std::vector<float>{euler_angles.roll, euler_angles.pitch, euler_angles.yaw});
	}
}

void SpecificWorker::change_rt_from_robot_to_room()
{
	G->delete_edge("robot","bottle","RT");

	auto room_node_opt = G->get_node("room");
	auto bottle_node_opt = G->get_node("bottle");
	if(!room_node_opt.has_value() || !bottle_node_opt.has_value()){
		std::cout << "Either room or bottle node is missing in the graph." << std::endl;
		return;
	}
	DSR::Node room_node = room_node_opt.value();
	DSR::Node bottle_node = bottle_node_opt.value();

	DSR::Edge loss_edge;
	loss_edge.from(room_node.id());
	loss_edge.to(bottle_node.id());
	loss_edge.type("RT");
	
	G->insert_or_assign_edge(loss_edge);
}

void SpecificWorker::eliminate_rt_from_robot_to_bottle()
{
	G->delete_edge("robot","bottle","RT");
}


/**************************************/
/********* DETECTION METHODS **********/
SpecificWorker::BottlePose SpecificWorker::compute_bottle_orientation(const RoboCompImageSegmentation::PointCloud& point_cloud)
{
	SpecificWorker::BottlePose result{0, 0, 0, 0, 0, 0, 1.0, 0};
	
	if (point_cloud.numberPoints < 3)
		return result;

	const int n = point_cloud.numberPoints;

	// ── 1. Calcular centroide ──────────────────────────────────────────────
	float cx = 0, cy = 0, cz = 0;

	cx = std::accumulate(point_cloud.X.begin(), point_cloud.X.end(), 0.0f);
	cy = std::accumulate(point_cloud.Y.begin(), point_cloud.Y.end(), 0.0f);
	cz = std::accumulate(point_cloud.Z.begin(), point_cloud.Z.end(), 0.0f);

	cx /= n;
	cy /= n;
	cz /= n;
	
	result.x = cx;
	result.y = cy;
	result.z = cz;

	// ── 2. Calcular matriz de covarianza ───────────────────────────────────
	double cxx = 0, cyy = 0, czz = 0, cxy = 0, cxz = 0, cyz = 0;
	for (int i = 0; i < n; ++i)
	{
		float dx = point_cloud.X[i] - cx;
		float dy = point_cloud.Y[i] - cy;
		float dz = point_cloud.Z[i] - cz;
		cxx += dx * dx;
		cyy += dy * dy;
		czz += dz * dz;
		cxy += dx * dy;
		cxz += dx * dz;
		cyz += dy * dz;
	}
	cxx /= n;
	cyy /= n;
	czz /= n;
	cxy /= n;
	cxz /= n;
	cyz /= n;

	// ── 3. Jacobi eigendecomposition (3x3 simétrica) ───────────────────────
	double A[3][3] = {{cxx, cxy, cxz}, {cxy, cyy, cyz}, {cxz, cyz, czz}};
	double V[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

	// Iteraciones de Jacobi
	for (int iter = 0; iter < 50; ++iter)
	{
		int p = 0, q = 1;
		double maxv = std::abs(A[0][1]);
		if (std::abs(A[0][2]) > maxv)
		{
			maxv = std::abs(A[0][2]);
			p = 0;
			q = 2;
		}
		if (std::abs(A[1][2]) > maxv)
		{
			maxv = std::abs(A[1][2]);
			p = 1;
			q = 2;
		}
		if (maxv < 1e-10)
			break;

		double theta = 0.5 * std::atan2(2 * A[p][q], A[q][q] - A[p][p]);
		double c = std::cos(theta), s = std::sin(theta);

		double Anew[3][3];
		std::memcpy(Anew, A, sizeof(A));
		Anew[p][p] = c * c * A[p][p] + 2 * s * c * A[p][q] + s * s * A[q][q];
		Anew[q][q] = s * s * A[p][p] - 2 * s * c * A[p][q] + c * c * A[q][q];
		Anew[p][q] = Anew[q][p] = 0;
		for (int r = 0; r < 3; ++r)
		{
			if (r == p || r == q)
				continue;
			Anew[r][p] = Anew[p][r] = c * A[r][p] + s * A[r][q];
			Anew[r][q] = Anew[q][r] = -s * A[r][p] + c * A[r][q];
		}
		std::memcpy(A, Anew, sizeof(A));
		for (int r = 0; r < 3; ++r)
		{
			double vp = V[r][p], vq = V[r][q];
			V[r][p] = c * vp + s * vq;
			V[r][q] = -s * vp + c * vq;
		}
	}

	// ── 4. Ordenar por eigenvalues (descendente) ──────────────────────────
	int order[3] = {0, 1, 2};
	if (A[order[0]][order[0]] < A[order[1]][order[1]])
		std::swap(order[0], order[1]);
	if (A[order[0]][order[0]] < A[order[2]][order[2]])
		std::swap(order[0], order[2]);
	if (A[order[1]][order[1]] < A[order[2]][order[2]])
		std::swap(order[1], order[2]);

	// ── 5. Obtener eje principal (primer eigenvector - altura de la botella) ─
	float principal_axis[3] = {static_cast<float>(V[0][order[0]]),
	                            static_cast<float>(V[1][order[0]]),
	                            static_cast<float>(V[2][order[0]])};

	// Normalizar
	float norm = std::sqrt(principal_axis[0] * principal_axis[0] +
	                        principal_axis[1] * principal_axis[1] +
	                        principal_axis[2] * principal_axis[2]);
	if (norm > 0.001f)
	{
		principal_axis[0] /= norm;
		principal_axis[1] /= norm;
		principal_axis[2] /= norm;
	}

	// ── 6. Calcular ángulo de inclinación respecto a Z (vertical) ──────────
	float z_axis[3] = {0, 0, 1};
	float dot_product = principal_axis[0] * z_axis[0] +
	                     principal_axis[1] * z_axis[1] +
	                     principal_axis[2] * z_axis[2];
	
	// Limitar a [-1, 1] para evitar errores numéricos
	dot_product = std::max(-1.0f, std::min(1.0f, dot_product));
	result.tilt_angle = std::acos(std::abs(dot_product)) * 180.0f / M_PI;

	// ── 7. Calcular quaternion de rotación ───────────────────────────────
	// Vector perpendicular entre principal_axis y z_axis
	float cross[3] = {principal_axis[1] * z_axis[2] - principal_axis[2] * z_axis[1],
	                   principal_axis[2] * z_axis[0] - principal_axis[0] * z_axis[2],
	                   principal_axis[0] * z_axis[1] - principal_axis[1] * z_axis[0]};

	float cross_norm = std::sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);

	if (cross_norm > 0.001f)
	{
		cross[0] /= cross_norm;
		cross[1] /= cross_norm;
		cross[2] /= cross_norm;

		float angle = std::acos(dot_product);
		float sin_half = std::sin(angle / 2.0f);
		float cos_half = std::cos(angle / 2.0f);

		result.qx = cross[0] * sin_half;
		result.qy = cross[1] * sin_half;
		result.qz = cross[2] * sin_half;
		result.qw = cos_half;
	}
	else
	{
		result.qw = 1.0;
	}

	// ── 8. Calcular ángulos de Euler (aproximado) ─────────────────────────
	float euler_x = std::atan2(principal_axis[1], principal_axis[2]) * 180.0f / M_PI;
	float euler_y = std::atan2(-principal_axis[0], 
	                             std::sqrt(principal_axis[1] * principal_axis[1] + 
	                                      principal_axis[2] * principal_axis[2])) * 180.0f / M_PI;

	// ── 9. Calcular dimensiones proyectando puntos sobre ejes principales ──
	std::array<std::vector<float>, 3> projections;
	for (auto& proj : projections)
		proj.reserve(n);

	// Proyectar puntos sobre los 3 ejes principales
	for (int i = 0; i < n; ++i)
	{
		float dx = point_cloud.X[i] - cx;
		float dy = point_cloud.Y[i] - cy;
		float dz = point_cloud.Z[i] - cz;
		
		for (int a = 0; a < 3; ++a)
		{
			float proj_val = dx * V[0][order[a]] + dy * V[1][order[a]] + dz * V[2][order[a]];
			projections[a].push_back(proj_val);
		}
	}

	// Calcular extents usando percentiles (2-98%)
	auto calculate_percentile_extent = [](std::vector<float> vals) {
		if (vals.empty()) return 0.0f;
		std::sort(vals.begin(), vals.end());
		size_t lo_idx = static_cast<size_t>(0.02f * (vals.size() - 1));
		size_t hi_idx = static_cast<size_t>(0.98f * (vals.size() - 1));
		return vals[hi_idx] - vals[lo_idx];
	};

	// Asignar dimensiones
	result.height = calculate_percentile_extent(projections[0]); // mm
	result.width = calculate_percentile_extent(projections[1]);
	result.depth = calculate_percentile_extent(projections[2]);

	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

	if(now - bottle_dimension_display_timer >= std::chrono::seconds(2))
	{
		bottle_dimension_display_timer = now;
		std::cout << "\n=== Bottle Pose ==="
				<< "\nPosition: (" << result.x / 10.0f << ", " << result.y / 10.0f 
				<< ", " << result.z / 10.0f << ") cm"
				<< "\nDimensions: H=" << result.height / 10.0f << " W=" << result.width / 10.0f
				<< " D=" << result.depth / 10.0f << " cm"
				<< "\nTilt: " << result.tilt_angle << "°" << std::endl;
	}

	return result;
}

bool SpecificWorker::validate_bottle(const RoboCompImageSegmentation::SegmentedObject& obj)
{
	if (obj.label != "bottle")
		return false;

	bool at_least_one_point_in_range = true; // Flag to check if at least one point is in the specified range

	for(int i = 0; i < obj.points3D.numberPoints; i++)
	{
		float x = obj.points3D.X[i];
		float y = obj.points3D.Y[i];
		float z = obj.points3D.Z[i];
		if (y >= 0 && y <= Y_range_max && x >= X_range_min && x <= X_range_max && z >= Z_range_min)
		{
			at_least_one_point_in_range = true; // Set the flag to true if at least one point is in the range
			break; // No need to check further points, we can consider this object as a valid bottle
		}
	}
	return at_least_one_point_in_range; // Return true if at least one point is in the range, false otherwise
}

RoboCompImageSegmentation::PointCloud SpecificWorker::filter_pointcloud(const RoboCompImageSegmentation::PointCloud& point_cloud)
{
	const int n = point_cloud.numberPoints;

    RoboCompImageSegmentation::PointCloud filtered_cloud;

    filtered_cloud.X.reserve(n);
    filtered_cloud.Y.reserve(n);
    filtered_cloud.Z.reserve(n);
	
	for(int i = 0; i < n; i++)
	{
		float x, y, z;
		if (simulated)
		{
			x = point_cloud.X[i] * 1000.0f; // Convertir a milímetros
			y = point_cloud.Y[i] * 1000.0f;
			z = point_cloud.Z[i] * 1000.0f;
		}
		else
		{
			x = point_cloud.X[i];
			y = point_cloud.Y[i];
			z = point_cloud.Z[i];
		}
		
		if (y >= 0 && y <= Y_range_max && x >= X_range_min && x <= X_range_max && z >= Z_range_min)
		{
				filtered_cloud.X.emplace_back(x);
				filtered_cloud.Y.emplace_back(y);
				filtered_cloud.Z.emplace_back(z);
		}
	}
	filtered_cloud.numberPoints = filtered_cloud.X.size();

	return filtered_cloud;
}


std::optional<SpecificWorker::BottlePose> SpecificWorker::get_bottle_pose_from_segmented_mask(const RoboCompImageSegmentation::SegmentedObject& obj)
{
	//Get the mask height and width
	RoboCompImageSegmentation::Polygon polygon = obj.imagePolygon;
	BottlePose pose;
	
	if (polygon.U.empty() || polygon.V.empty())
		return std::nullopt;

	// Calculate the bounding box of the polygon
	int min_x = *std::min_element(polygon.U.begin(), polygon.U.end());
	int max_x = *std::max_element(polygon.U.begin(), polygon.U.end());
	int min_y = *std::min_element(polygon.V.begin(), polygon.V.end());
	int max_y = *std::max_element(polygon.V.begin(), polygon.V.end());

	// Calculate the center of the bounding box
	int center_x = (min_x + max_x) / 2;
	int center_y = (min_y + max_y) / 2;

	//Calculate if the object has fallen based on the aspect ratio of the bounding box (if width is greater than height, we can assume it has fallen)
	int width = pose.height = max_x - min_x;
	int height = pose.width = max_y - min_y;

	if (height < 0 && width < 0)
		return std::nullopt;

	if (height > width)
		pose.tilt_angle = 0.0f; // Bottle is upright
	else
		pose.tilt_angle = 90.0f; // Bottle has fallen

	return std::make_optional(pose);
}


std::optional<SpecificWorker::BottlePose> SpecificWorker::detect_bottle()
{	
	SpecificWorker::BottlePose bottle_pose;

	auto objects = this->imagesegmentation_proxy->getSegmentedObjects(true, false);

	RoboCompImageSegmentation::SegmentedObject bottle_obj; 
	bool bottle_found = false; 

	for (const auto& obj : objects)
	{
		if (validate_bottle(obj)) 
		{
			bottle_obj = obj; 
			bottle_found = true; 
			break; 
		}
	}

	if(!bottle_found)
	{
		std::cout << "No bottle detected in image segmentation." << std::endl;
		return std::nullopt;
	}

	if (!use_point_cloud_segmentation)
		return get_bottle_pose_from_segmented_mask(bottle_obj);

	auto point_cloud = bottle_obj.points3D;

	point_cloud = filter_pointcloud(point_cloud);

	if (display_point_cloud)
	{
		points->clear(); 
		colors->clear();

		for (int i = 0; i < point_cloud.numberPoints; i++)
		{
			points->emplace_back(std::make_tuple(point_cloud.X[i] / 1000.0, point_cloud.Y[i] / 1000.0, point_cloud.Z[i] / 1000.0));
			colors->emplace_back(std::make_tuple(0.0f, 1.0f, 0.0f)); // Green points
		}
		viewer_3d->update();
	}

	if (point_cloud.numberPoints < 100) // Threshold for minimum number of points after filtering
	{
		std::cout << "Not enough points in the bottle point cloud after filtering." << std::endl;
		return std::nullopt;
	}

	// Calcular orientación e inclinación usando PCA
	bottle_pose = compute_bottle_orientation(point_cloud);
	
	return std::make_optional(bottle_pose);
}

bool SpecificWorker::bottle_has_fallen(const std::optional<BottlePose>& bottle_pose)
{
	// Will check for 5 consecutive frames if the bottle has fallen based on its tilt angle and orientation and if the bottle pose is valid
	if(!bottle_pose.has_value())
	{
		bottle_lost_count++;
		if (bottle_lost_count >= bottle_lost_threshold)
		{
			bottle_lost_count = 0; // Reset counter after confirming bottle has fallen
			std::cout << "Bottle has fallen! No valid pose detected for " << bottle_lost_threshold << " consecutive frames." << std::endl;
			return true;
		}
		return false;
	}

	if (bottle_pose->tilt_angle > tilt_threshold)
	{
		bottle_lost_count++;
		if (bottle_lost_count >= bottle_lost_threshold)
		{
			bottle_lost_count = 0; // Reset counter after confirming bottle has fallen
			std::cout << "Bottle has fallen! Tilt angle: " << bottle_pose->tilt_angle << "°" << std::endl;
			return true;
		}
		std::cout << "Bottle tilt detected: " << bottle_pose->tilt_angle << "°" << std::endl;
		return false;
	}

	bottle_lost_count = 0; // Reset counter if bottle is detected and not tilted
	return false;
}

/**************************************/
// From the RoboCompImageSegmentation you can call this methods:
// RoboCompImageSegmentation::TData this->imagesegmentation_proxy->getAll(bool points3d, bool rgb)
// RoboCompImageSegmentation::TDepth this->imagesegmentation_proxy->getDepth()
// RoboCompImageSegmentation::TImage this->imagesegmentation_proxy->getImage()
// RoboCompImageSegmentation::ObjectList this->imagesegmentation_proxy->getSegmentedObjects(bool points3d, bool rgb)

/**************************************/
// From the RoboCompImageSegmentation you can use this types:
// RoboCompImageSegmentation::PointCloud
// RoboCompImageSegmentation::Polygon
// RoboCompImageSegmentation::SegmentedObject
// RoboCompImageSegmentation::TImage
// RoboCompImageSegmentation::TDepth
// RoboCompImageSegmentation::TData

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompWebots2Robocomp you can call this methods:
// RoboCompWebots2Robocomp::ObjectPose this->webots2robocomp_proxy->getObjectPose(string DEF)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->resetWebots()
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setDoorAngle(float angle)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setPathToHuman(int humanId, RoboCompGridder::TPath path)

/**************************************/
// From the RoboCompWebots2Robocomp you can use this types:
// RoboCompWebots2Robocomp::Vector3
// RoboCompWebots2Robocomp::Quaternion
// RoboCompWebots2Robocomp::ObjectPose

