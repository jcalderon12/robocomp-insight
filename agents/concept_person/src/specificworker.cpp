/*
 *    Copyright (C) 2025 by YOUR NAME HERE
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

	/***
	Custom Widget
	In addition to the predefined viewers, Graph Viewer allows you to add various widgets designed by the developer.
	The add_custom_widget_to_dock method is used. This widget can be defined like any other Qt widget,
	either with a QtDesigner or directly from scratch in a class of its own.
	The add_custom_widget_to_dock method receives a name for the widget and a reference to the class instance.
	***/

	//graph_viewers.at("")->add_custom_widget_to_dock("CustomWidget", &custom_widget);

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

	rt = G->get_rt_api();

	// Paseo de la persona: ruta "Start" (una vez) + "Loop" (en bucle), numeros separados por comas
	auto split_route = [this](const std::string &list) {
		std::vector<std::string> defs;
		std::stringstream ss(list);
		for (std::string token; std::getline(ss, token, ',');)
		{
			token.erase(0, token.find_first_not_of(" \t"));
			token.erase(token.find_last_not_of(" \t") + 1);
			if (!token.empty())
				defs.push_back(walk_prefix + token);
		}
		return defs;
	};
	try { walk_enabled = configLoader.get<bool>("Person.Walk.Enabled"); } catch (...) {}
	try { walk_speed = static_cast<float>(configLoader.get<double>("Person.Walk.Speed")); } catch (...) {}
	try { walk_prefix = configLoader.get<std::string>("Person.Walk.Prefix"); } catch (...) {}
	std::string start_list, loop_list;
	try { start_list = configLoader.get<std::string>("Person.Walk.Start"); } catch (...) {}
	try { loop_list = configLoader.get<std::string>("Person.Walk.Loop"); } catch (...) {}
	walk_route = split_route(start_list);
	walk_loop_start = walk_route.size();
	for (const auto &def : split_route(loop_list))
		walk_route.push_back(def);
	if (walk_enabled && walk_route.empty())
	{
		std::cout << "[paseo] Person.Walk.Enabled sin waypoints en Start/Loop: desactivado." << std::endl;
		walk_enabled = false;
	}
	if (walk_enabled)
	{
		std::cout << "[paseo] ruta:";
		for (size_t i = 0; i < walk_route.size(); ++i)
			std::cout << (i == walk_loop_start ? " | " : " ") << walk_route[i];
		std::cout << " (bucle desde " << (walk_loop_start < walk_route.size() ? walk_route[walk_loop_start] : "-")
		          << "), " << walk_speed << " m/s" << std::endl;
	}

	last_relative_pose = get_person_relative_position();
}



void SpecificWorker::compute()
{
	auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	if (walk_enabled)
		walk_person();
	record_robot_pose();
	auto detected = compensate_detection_delay(get_person_relative_position());
	// Un salto sin confirmar no se publica ni entra en la memoria: mientras, se sigue la recordada.
	const bool is_detection = std::hypot(detected[0], detected[1]) >= PERSON_NULL_DISTANCE;
	const bool held = is_detection && !confirm_person_jump(detected);
	auto relative_position = apply_person_memory(held ? std::vector<float>{0.f, 0.f, 0.f} : detected, held);

	if(print_extra_info)
		std::cout << "[" << now_ms << "] get_person_relative_position -> x: " << relative_position[0]
			<< " | y: " << relative_position[1] << " | z: " << relative_position[2] << std::endl;

	bool changed = has_significant_change(relative_position, last_relative_pose);
	if(print_extra_info)
		std::cout << "[" << now_ms << "] Significant change detected: " << (changed ? "Yes" : "No") << std::endl;

	if (changed){
		bool updated = update_relative_position_to_person(relative_position, now_ms);
		if(print_extra_info)
			std::cout << "[" << now_ms << "] update_relative_position_to_person -> " << (updated ? "Updated successfully" : "Update failed") << std::endl;
		if (updated){
			last_relative_pose = relative_position;
		}
	}

	switch (agentState){
		case States::IDLE:
		{
			bool is_target_assigned = check_target_assigned();
			if (is_target_assigned){
				std::cout << "Target assigned to person" << std::endl;
				agentState = States::ASSIGNED;
			}
			break;
		}

		case States::ASSIGNED:
		{
			bool affordance_existed = check_affordance_assigned();
			if (!affordance_existed)
				create_affordance();
			bool affordance_acepted = check_affordance_accepted();
			if (affordance_acepted){
				std::cout << "Affordance accepted for the assigned target." << std::endl;
				agentState = States::FOLLOWME;
			}
			break;
		}

		case States::FOLLOWME:
		{
			bool affordance_state = check_affordance_active();
			if (affordance_state == false){
				std::cout << "Mission finished" << std::endl;
				agentState = States::SUCCESS;
			}
			break;
		}

		case States::SUCCESS:
			break;

		case States::FAILED:
			break;

		case States::STOPPED:
			break;
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

/******* Geometric calculations ********/

bool SpecificWorker::has_significant_change(const std::vector<float>& a,
                                            const std::vector<float>& b,
                                            double atol)
{
    if (a.size() != b.size())
        return true;

    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > atol) {
            return true; 
        }
    }
    return false;
}

std::vector<float> SpecificWorker::get_person_relative_position()
{
	std::vector<float> relative_position = {0.0f, 0.0f, 0.0f};

	if (simulated){
		auto person_pose = this->webots2robocomp_proxy->getObjectPose(person_def);
		auto robot_pose  = this->webots2robocomp_proxy->getObjectPose(robot_def);

		Eigen::Vector3f p_pos(person_pose.position.x / 1000.f,
		                      person_pose.position.y / 1000.f,
		                      person_pose.position.z / 1000.f);
		Eigen::Vector3f r_pos(robot_pose.position.x / 1000.f,
		                      robot_pose.position.y / 1000.f,
		                      robot_pose.position.z / 1000.f);
		Eigen::Quaternionf r_quat(robot_pose.orientation.w, robot_pose.orientation.x,
		                          robot_pose.orientation.y, robot_pose.orientation.z);
		r_quat.normalize();

		relative_position = {p_pos[0] - r_pos[0], p_pos[1] - r_pos[1], p_pos[2] - r_pos[2]};

		float dx = (person_pose.position.x - robot_pose.position.x) / 1000.f;
		float dy = (person_pose.position.y - robot_pose.position.y) / 1000.f;
		float dz = (person_pose.position.z - robot_pose.position.z) / 1000.f;

		Eigen::Quaternionf q(robot_pose.orientation.w, robot_pose.orientation.x,
							 robot_pose.orientation.y, robot_pose.orientation.z);

		q.normalize();
		// float theta = q.toRotationMatrix().eulerAngles(0, 1, 2)[2];
		float theta = std::atan2(2.f * (q.w() * q.z() + q.x() * q.y()), 1.f - 2.f * (q.y() * q.y() + q.z() * q.z()));
		float local_x = -std::sin(theta) * dx + std::cos(theta) * dy;
		float local_y = -std::cos(theta) * dx - std::sin(theta) * dy;

		relative_position = {local_y, local_x, dz};

		// debug
		auto ts_person = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
		if(print_extra_info)
			std::cout << "[" << ts_person << "] local_delta -> dx: " << relative_position[0]
					<< " | dy: " << relative_position[1]
					<< " | theta: " << theta << std::endl;
	}
	else{
		// getAll en vez de getSegmentedObjects: trae la hora de captura de la imagen (image.alivetime).
		auto data = this->imagesegmentation_proxy->getAll(true, false);
		const auto &segmented_objects = data.objects;
		detection_capture_ms = data.image.alivetime;

		std::vector<std::vector<float>> person_positions;

		for (const auto& obj : segmented_objects){
			if (obj.label == "person"){
				auto point_cloud = obj.points3D;
				if (!point_cloud.X.empty()){
					person_positions.push_back(median_point(point_cloud));
				}
			}
		}

		if (!person_positions.empty()){
			// Tolerance in mm: if two people are within this distance, prefer the one more centered
			const float distance_tolerance_mm = 150.0f; 
			auto best_person = person_positions[0];
			auto best_distance_sq = best_person[0]*best_person[0] + best_person[1]*best_person[1] + best_person[2]*best_person[2];

			auto best_center_offset = std::abs(best_person[0]);

			for (const auto& pos : person_positions){
				float distance_sq = pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2];
				// If clearly closer (beyond tolerance), this is the new best
				if (distance_sq + distance_tolerance_mm * distance_tolerance_mm < best_distance_sq){
					best_person = pos;
					best_distance_sq = distance_sq;
					best_center_offset = std::abs(pos[0]);
				}
				// If within tolerance (similar distance), prefer the more centered one
				else if (std::abs(distance_sq - best_distance_sq) <= distance_tolerance_mm * distance_tolerance_mm){
					auto center_offset = std::abs(pos[0]);
					// Prefer less offset from center; if tied, prefer less vertical offset
					if (center_offset < best_center_offset ||
					    (center_offset == best_center_offset && std::abs(pos[1]) < std::abs(best_person[1]))){
						best_person = pos;
						best_center_offset = center_offset;
					}
				}
			}

			relative_position = {best_person[0], best_person[1], best_person[2]};
		}
	}

	return relative_position;
}


std::vector<float> SpecificWorker::median_point(const RoboCompImageSegmentation::PointCloud& cloud)
{
	// Mediana por eje: un punto suelto del fondo no arrastra la posicion como lo hacia el primero.
	auto median = [](std::vector<float> v) {
		const auto mid = v.begin() + v.size() / 2;
		std::nth_element(v.begin(), mid, v.end());
		return *mid;
	};
	std::vector<float> xs, ys, zs;
	const size_t n = std::min({cloud.X.size(), cloud.Y.size(), cloud.Z.size()});
	for (size_t i = 0; i < n; ++i)
		if (std::isfinite(cloud.X[i]) && std::isfinite(cloud.Y[i]) && std::isfinite(cloud.Z[i]) && cloud.Y[i] > 0.f)
		{
			xs.push_back(cloud.X[i]);
			ys.push_back(cloud.Y[i]);
			zs.push_back(cloud.Z[i]);
		}
	if (xs.empty())
		return {0.f, 0.f, 0.f};   // ausencia de dato
	return {median(xs), median(ys), median(zs)};
}

std::vector<float> SpecificWorker::apply_person_memory(const std::vector<float>& detected, bool held)
{
	float robot_x = 0.f, robot_y = 0.f, yaw = 0.f;
	const bool have_pose = get_robot_pose_2d(robot_x, robot_y, yaw);
	const float c = std::cos(yaw), s = std::sin(yaw);
	auto now = std::chrono::steady_clock::now();

	// Ejes del robot en el marco root, medidos en la grabacion de follow_person del 03/11/2026:
	// el robot avanza en (cos yaw, -sin yaw) y su derecha es (sin yaw, cos yaw). auto_localization()
	// intercambia x/y de la traslacion pero deja el cuaternion de Webots, de ahi el signo cambiado.
	if (std::hypot(detected[0], detected[1]) >= PERSON_NULL_DISTANCE)
	{
		set_person_perceived(true);
		if (have_pose)
		{
			// detected = {derecha, avance, altura} en el marco del robot.
			person_memory_x = robot_x + detected[1] * c + detected[0] * s;
			person_memory_y = robot_y - detected[1] * s + detected[0] * c;
			person_memory_z = detected[2];
			person_memory_stamp = now;
			person_memory_valid = true;
			person_memory_expired_warned = false;
		}
		return detected;
	}

	// Un salto retenido no es perder a la persona: perceived se queda como estaba.
	if (!held)
		set_person_perceived(false);
	const float age = std::chrono::duration<float>(now - person_memory_stamp).count();
	if (!person_memory_valid || !have_pose || age > PERSON_MEMORY_MAX_AGE)
	{
		if (person_memory_valid && age > PERSON_MEMORY_MAX_AGE && !person_memory_expired_warned)
		{
			std::cout << "Memoria de la persona caducada (" << age << " s); se publica ausencia de dato." << std::endl;
			person_memory_expired_warned = true;
		}
		return detected;   // ceros: ausencia de dato
	}

	// Posicion recordada vista desde la pose actual: el robot se gira hacia donde la vio por ultima vez.
	const float dx = person_memory_x - robot_x;
	const float dy = person_memory_y - robot_y;
	return {dx * s + dy * c, dx * c - dy * s, person_memory_z};
}

void SpecificWorker::record_robot_pose()
{
	float x, y, yaw;
	if (!get_robot_pose_2d(x, y, yaw))
		return;
	const long long now = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count();
	robot_pose_history.push_back({now, x, y, yaw});
	while (!robot_pose_history.empty() && now - robot_pose_history.front().ms > POSE_HISTORY_MS)
		robot_pose_history.pop_front();
}

std::vector<float> SpecificWorker::compensate_detection_delay(const std::vector<float>& detected)
{
	// Deteccion = {derecha, avance, altura} en el marco del robot cuando se capturo la imagen. Se pasa al
	// marco root con la pose de entonces (interpolada del historial) y se vuelve con la actual.
	if (simulated || std::hypot(detected[0], detected[1]) < PERSON_NULL_DISTANCE || robot_pose_history.size() < 2)
		return detected;
	const long long now = robot_pose_history.back().ms;
	const long long age = now - detection_capture_ms;
	if (age < 0 || age > MAX_DETECTION_AGE_MS)
	{
		if (!delay_age_warned)
			std::cout << "[retardo] edad de la captura fuera de rango (" << age << " ms); deteccion sin compensar." << std::endl;
		delay_age_warned = true;
		return detected;
	}
	delay_age_warned = false;
	// Pose en la captura: interpolada entre las dos muestras que la rodean (o la mas antigua)
	StampedPose then = robot_pose_history.front();
	for (size_t i = 1; i < robot_pose_history.size(); ++i)
	{
		const auto &a = robot_pose_history[i - 1], &b = robot_pose_history[i];
		if (b.ms < detection_capture_ms)
			continue;
		const float f = b.ms > a.ms ? std::clamp(static_cast<float>(detection_capture_ms - a.ms) / (b.ms - a.ms), 0.f, 1.f) : 1.f;
		const float dyaw = std::remainder(b.yaw - a.yaw, 2.f * static_cast<float>(M_PI));
		then = {detection_capture_ms, a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), a.yaw + f * dyaw};
		break;
	}
	const auto &cur = robot_pose_history.back();
	// Mismos ejes que apply_person_memory(): avance (cos yaw, -sin yaw), derecha (sin yaw, cos yaw)
	const float c0 = std::cos(then.yaw), s0 = std::sin(then.yaw);
	const float wx = then.x + detected[1] * c0 + detected[0] * s0;
	const float wy = then.y - detected[1] * s0 + detected[0] * c0;
	const float c1 = std::cos(cur.yaw), s1 = std::sin(cur.yaw);
	const float dx = wx - cur.x, dy = wy - cur.y;
	std::vector<float> out = {dx * s1 + dy * c1, dx * c1 - dy * s1, detected[2]};

	// Traza: como mucho una por segundo, solo si el robot se ha movido de forma apreciable
	const float turned = std::remainder(cur.yaw - then.yaw, 2.f * static_cast<float>(M_PI)) * 180.f / static_cast<float>(M_PI);
	const float shift = std::hypot(out[0] - detected[0], out[1] - detected[1]);
	const auto steady_now = std::chrono::steady_clock::now();
	if (shift > 0.05f && std::chrono::duration<float>(steady_now - delay_trace_last).count() > 1.f)
	{
		delay_trace_last = steady_now;
		std::cout << "[retardo] captura hace " << age << " ms; el robot giro " << turned << " grados: deteccion ("
		          << detected[0] << ", " << detected[1] << ") -> (" << out[0] << ", " << out[1] << "), " << shift << " m" << std::endl;
	}
	return out;
}

bool SpecificWorker::get_robot_pose_2d(float &x, float &y, float &yaw)
{
	auto robot_opt = G->get_node("robot");
	auto root_opt = G->get_node("root");
	if (!robot_opt.has_value() || !root_opt.has_value())
		return false;

	auto t_opt = rt->get_translation(root_opt.value(), robot_opt.value().id());
	auto edge_opt = rt->get_edge_RT(root_opt.value(), robot_opt.value().id());
	if (!t_opt.has_value() || !edge_opt.has_value())
		return false;
	auto q_opt = G->get_attrib_by_name<rt_quaternion_att>(edge_opt.value());
	if (!q_opt.has_value())
		return false;
	const std::vector<float> &q = q_opt.value().get();   // {qx, qy, qz, qw}
	if (q.size() < 4)
		return false;
	x = static_cast<float>(t_opt.value().x());
	y = static_cast<float>(t_opt.value().y());
	yaw = std::atan2(2.f * (q[3] * q[2] + q[0] * q[1]), 1.f - 2.f * (q[1] * q[1] + q[2] * q[2]));
	return true;
}

void SpecificWorker::set_person_perceived(bool perceived)
{
	if (person_perceived_published == static_cast<int>(perceived))
		return;
	auto person_opt = G->get_node("person");
	if (!person_opt.has_value())
		return;

	DSR::Node person_node = person_opt.value();
	G->runtime_checked_add_or_modify_attrib_local(person_node, "perceived", perceived);
	G->update_node(person_node);
	person_perceived_published = static_cast<int>(perceived);
	if (!perceived)
		std::cout << "Persona fuera de la vista: se sigue su ultima posicion conocida." << std::endl;
}

float SpecificWorker::calculate_linear_speed_from_distance(std::vector<float> distance)
{
	const float min_distance = 1.0;
	const float K = 100;

	const float distance_2d = std::sqrt(
		distance[0] * distance[0] + 
		distance[1] * distance[1] +
		distance[2] * distance[2]
	);

	if(distance_2d <= min_distance){
		return 0.0;
	}

	float error = distance_2d - min_distance;

	return WEBOTS_MAX_LINEAR_SPEED * std::tanh(K * error);
}

/************ DSR Methods *************/

bool SpecificWorker::check_target_assigned()
{
	auto optional_robot_node = G->get_node("robot");
	if (!optional_robot_node.has_value()){
		if(print_extra_info == true)
			std::cout << "Robot node not found in DSR." << std::endl;
		return false;
	}
	auto optional_person_node = G->get_node("person");
	if (!optional_person_node.has_value()){
		if(print_extra_info == true)
			std::cout << "No person node found in DSR." << std::endl;
		return false;
	}

	auto target_edges = G->get_edges_by_type("TARGET");
	if (target_edges.empty()){
		if(print_extra_info == true)
			std::cout << "No TARGET edge found from robot to person." << std::endl;
		return false;
	}
	
	return true;
}

bool SpecificWorker::check_affordance_assigned()
{
	auto optional_person_node = G->get_node("person");
	if (!optional_person_node.has_value()){
		if (print_extra_info)
			std::cout << "No person node found in DSR." << std::endl;
		return false;
	}

	auto person_node = optional_person_node.value();

	auto affordance_nodes = G->get_nodes_by_type("affordance");
	for (const auto& node : affordance_nodes){
		auto edge = G->get_edge(person_node.id(), node.id(), "has_intention");
		if (edge.has_value()){
			if(print_extra_info)
				std::cout << "Affordance already exists for the assigned target." << std::endl;
			return true;
		}
	}
	if (print_extra_info)
		std::cout << "No affordance exists for the assigned target." << std::endl;
	return false;
}

void SpecificWorker::create_affordance()
{
	auto optional_person_node = G->get_node("person");
	if (!optional_person_node.has_value()){
		std::cout << "No person node found in DSR." << std::endl;
		return;
	}

	auto person_node = optional_person_node.value();

	std::cout << "Creating affordance for the assigned target." << std::endl;
	DSR::Node affordance_node = DSR::Node::create<affordance_node_type>("follow_me");
	auto pos_x = G->get_attrib_by_name<pos_x_att>(person_node.id()).value();
	auto pos_y = G->get_attrib_by_name<pos_y_att>(person_node.id()).value();
	pos_y = pos_y - 100.0; //Position the affordance slightly above the person
	G->add_or_modify_attrib_local<pos_x_att>(affordance_node, pos_x);
    G->add_or_modify_attrib_local<pos_y_att>(affordance_node, pos_y);
	G->add_or_modify_attrib_local<parent_att>(affordance_node, person_node.id());
	G->add_or_modify_attrib_local<aff_interacting_att>(affordance_node, false);
	// Mision que atiende esta afordance (atributo sin registrar en el nucleo: va por el mapa)
	DSR::Attribute mission_attr;
	mission_attr.value(std::string("follow_person"));
	affordance_node.attrs()["aff_mission"] = mission_attr;
	G->insert_node(affordance_node);
	G->update_node(affordance_node);

	DSR::Edge affordance_edge;
	affordance_edge.from(person_node.id());
	affordance_edge.to(affordance_node.id());
	affordance_edge.type("has_intention");
	G->insert_or_assign_edge(affordance_edge);
	std::cout << "Waiting for affordance acceptance..." << std::endl;
}

bool SpecificWorker::check_affordance_accepted()
{
	if (auto optional_node = G->get_node("follow_me"); !optional_node.has_value()){
		std::cout << "Affordance node not found in DSR." << std::endl;
		return false;
	}
	else{
		DSR::Node affordance_node = optional_node.value();
		auto interacting = G->get_attrib_by_name<aff_interacting_att>(affordance_node.id());
		if (interacting.has_value() && interacting.value() == true){
			return true;
		}
		else
			return false;
	}
}

bool SpecificWorker::update_relative_position_to_person(const std::vector<float>& relative_position, std::uint64_t now_ms)
{
	auto optional_robot_node = G->get_node("robot");
    auto optional_person_node = G->get_node("person");

    if (!optional_robot_node || !optional_person_node)
    {
        std::cout << "Robot or Person node not found in DSR." << std::endl;
        return false;
    }

	if (relative_position.size() != 3)
	{
		std::cout << "Invalid relative position size." << std::endl;
		return false;
	}

	auto robot_node = optional_robot_node.value();
	auto person_node = optional_person_node.value();
	rt->insert_or_assign_edge_RT(robot_node, person_node.id(), relative_position, {0.0f, 0.0f, 0.0f});
	last_relative_pose = relative_position;
	return true;
}

bool SpecificWorker::confirm_person_jump(const std::vector<float>& relative_position)
{
	float dx = relative_position[0] - last_relative_pose[0];
	float dy = relative_position[1] - last_relative_pose[1];
	float dz = relative_position[2] - last_relative_pose[2];
	float jump_distance = std::sqrt(dx*dx + dy*dy + dz*dz);

	if (jump_distance > large_jump_threshold)
	{
		float pdx = relative_position[0] - pending_relative_position[0];
		float pdy = relative_position[1] - pending_relative_position[1];
		float pdz = relative_position[2] - pending_relative_position[2];
		float pending_distance = std::sqrt(pdx*pdx + pdy*pdy + pdz*pdz);

		if (pending_distance > large_jump_candidate_tolerance)
		{
			pending_relative_position = relative_position;
			consecutive_large_jump_confirmations = 1;
			if (print_extra_info)
				std::cout << "Large jump detected, waiting for confirmation (1/" << required_large_jump_confirmations << ")" << std::endl;
			return false;
		}
		else
		{
			consecutive_large_jump_confirmations++;
			if (print_extra_info){
				// std::cout << "Large jump confirmation " << consecutive_large_jump_confirmations << "/" << required_large_jump_confirmations << std::endl;
				std::cout << "jump_distance: " << jump_distance
				<< " | threshold: " << large_jump_threshold
				<< " | pending_distance: " << pending_distance
				<< " | tolerance: " << large_jump_candidate_tolerance
				<< " | confirmations: " << consecutive_large_jump_confirmations
				<< "/" << required_large_jump_confirmations << std::endl;

			}
			if (consecutive_large_jump_confirmations < required_large_jump_confirmations)
				return false;
		}
	}

	consecutive_large_jump_confirmations = 0;
	pending_relative_position = relative_position;
	return true;
}



bool SpecificWorker::check_affordance_active()
{
	auto target_edges = G->get_edges_by_type("TARGET");
	auto has_intention_edges = G->get_edges_by_type("has_intention");
	for (const auto& target_edge : target_edges)
	{
		for (const auto& intention_edge : has_intention_edges)
		{
			if (intention_edge.from() == target_edge.to())
			{
				auto affordance_node_opt = G->get_node(intention_edge.to());
				if (affordance_node_opt.has_value())
				{
					DSR::Node affordance_node = affordance_node_opt.value();
					bool aff_interacting = G->get_attrib_by_name<aff_interacting_att>(affordance_node.id()).value();
					
					return aff_interacting;
				}
			}
		}
	}
	return false;
}

bool SpecificWorker::follow_person(std::vector<float> distance)
{	
	auto optional_robot_node = G->get_node("robot");
	if (!optional_robot_node.has_value()){
		if (print_extra_info)
			std::cout << "Robot node not found in DSR." << std::endl;
		return false;
	}

	auto optional_person_node = G->get_node("person");
	if (!optional_person_node.has_value()){
		if (print_extra_info)
			std::cout << "Person node not found in DSR." << std::endl;
		return false;
	}

	auto optional_affordance_node = G->get_node("follow_me");
	if (!optional_affordance_node.has_value()){
		if (print_extra_info)
			std::cout << "Affordance node not found in DSR." << std::endl;
		return false;
	}

	auto robot_node = optional_robot_node.value();
	auto person_node = optional_person_node.value();
	auto affordance_node = optional_affordance_node.value();

	auto interacting = G->get_attrib_by_name<aff_interacting_att>(affordance_node.id());
	if (!interacting.has_value() || interacting.value() == false){
		if (print_extra_info)
			std::cout << "Affordance interaction stopped by user." << std::endl;
		G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node, (float)0.0);
		G->update_node(robot_node);
		return false;
	}
	if (print_extra_info)
		std::cout << "Following person with id: " << person_node.id() << std::endl;

	auto linear_speed = calculate_linear_speed_from_distance(distance);

	G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node, (float)linear_speed);
	G->update_node(robot_node);

	return true;
}


/************ Paseo de la persona *************/

void SpecificWorker::refresh_waypoints()
{
	for (const auto &def : walk_route)
	{
		if (waypoints.contains(def))
			continue;   // repetido en la ruta
		try
		{
			const auto pose = webots2robocomp_proxy->getObjectPose(def);
			// [0,0,0] es ausencia de dato: el puente devuelve ceros si no resuelve el DEF
			if (pose.position.x == 0.f && pose.position.y == 0.f && pose.position.z == 0.f)
				waypoints.erase(def);
			else
				waypoints[def] = {pose.position.x / 1000.f, pose.position.y / 1000.f};
		}
		catch (const Ice::Exception &e) { std::cout << "[paseo] sin pose de " << def << ": " << e.what() << std::endl; return; }
	}
}

void SpecificWorker::walk_person()
{
	// Activo solo con la afordance follow_me aceptada (aff_interacting = true)
	bool active = false;
	if (auto aff = G->get_node("follow_me"); aff.has_value())
	{
		auto interacting = G->get_attrib_by_name<aff_interacting_att>(aff.value());
		active = interacting.has_value() && interacting.value();
	}
	const auto now = std::chrono::steady_clock::now();

	if (active != walk_active)
	{
		walk_active = active;
		if (!active)
		{
			std::cout << "[paseo] pausado en " << walk_route[walk_index] << std::endl;
			return;
		}
		// Arranque o reanudacion: se parte de la pose real (puede haberse movido a mano) y se releen todos
		try
		{
			const auto pose = webots2robocomp_proxy->getObjectPose(person_def);
			if (pose.position.x == 0.f && pose.position.y == 0.f && pose.position.z == 0.f)
			{
				walk_active = false;   // ausencia de dato: se reintenta en el siguiente ciclo
				return;
			}
			walk_position = {pose.position.x / 1000.f, pose.position.y / 1000.f};
			walk_z = pose.position.z / 1000.f;
		}
		catch (const Ice::Exception &e) { std::cout << "[paseo] sin pose de " << person_def << ": " << e.what() << std::endl; walk_active = false; return; }
		waypoints.clear();
		refresh_waypoints();
		waypoints_last_refresh = now;
		walk_last_tick = now;
		std::cout << "[paseo] activo: hacia " << walk_route[walk_index] << std::endl;
		return;
	}
	if (!active)
		return;

	// Relectura periodica de toda la ruta: mover un waypoint en Webots se aplica en marcha
	if (std::chrono::duration<float>(now - waypoints_last_refresh).count() > WAYPOINT_REFRESH_PERIOD)
	{
		waypoints.clear();
		refresh_waypoints();
		waypoints_last_refresh = now;
	}

	const float dt = std::min(std::chrono::duration<float>(now - walk_last_tick).count(), WALK_MAX_DT);
	walk_last_tick = now;

	const auto &target_def = walk_route[walk_index];
	auto target_it = waypoints.find(target_def);
	if (target_it == waypoints.end())
	{
		if (!walk_missing_warned)
			std::cout << "[paseo] no existe " << target_def << " en Webots (o esta en el origen): esperando." << std::endl;
		walk_missing_warned = true;
		return;
	}
	walk_missing_warned = false;

	// Avance en linea recta a velocidad fija; al alcanzarlo, siguiente de la ruta (Loop vuelve a su inicio)
	const Eigen::Vector2f delta = target_it->second - walk_position;
	const float distance = delta.norm();
	const float step = walk_speed * dt;
	if (distance > 1e-3f)
		walk_heading = std::atan2(delta.y(), delta.x());
	if (distance <= step)
	{
		walk_position = target_it->second;
		// Sin Loop, al acabar Start se queda en el ultimo
		const size_t next = walk_index + 1 < walk_route.size() ? walk_index + 1
		                  : (walk_loop_start < walk_route.size() ? walk_loop_start : walk_index);
		if (next != walk_index)
		{
			std::cout << "[paseo] llegada a " << target_def << " -> " << walk_route[next] << std::endl;
			walk_index = next;
		}
	}
	else
		walk_position += delta / distance * step;

	// Orientacion: giro sobre z hacia donde camina (el modelo mira a +x, como en crowd_controller)
	RoboCompWebots2Robocomp::ObjectPose pose;
	pose.position = {walk_position.x() * 1000.f, walk_position.y() * 1000.f, walk_z * 1000.f};
	pose.orientation = {0.f, 0.f, std::sin(walk_heading / 2.f), std::cos(walk_heading / 2.f)};
	try { webots2robocomp_proxy->setObjectPose(person_def, pose); }
	catch (const Ice::Exception &e) { std::cout << "[paseo] setObjectPose fallo: " << e.what() << std::endl; }
}


//SUBSCRIPTION to setVisualObjects method from VisualElementsPub interface
void SpecificWorker::VisualElementsPub_setVisualObjects(RoboCompVisualElementsPub::TData data)
{
//subscribesToCODE

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

/**************************************/
// From the RoboCompVisualElementsPub you can use this types:
// RoboCompVisualElementsPub::TObject
// RoboCompVisualElementsPub::TData

