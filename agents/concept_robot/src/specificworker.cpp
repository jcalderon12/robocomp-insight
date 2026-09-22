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
#include <filesystem>
#include <fstream>
#include <sstream>
#include <limits>

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	setlocale(LC_NUMERIC, "C");
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

	last_velocities_readed = getVelocitiesFromDSR();

	prev_distance_error = 0.0f;
	prev_angle_error    = 0.0f;
	last_follow_time    = std::chrono::steady_clock::now();

	last_odometry = {0.0f, 0.0f, 0.0f};

	if (simulated)
		{
			desired_distance = configLoader.get<double>("Desired_distance") / 1000;
			photo_distance = configLoader.get<double>("Photo_distance") / 1000;
			std::cout << "Desired distance (simulated): " << desired_distance << std::endl;
		}
	else
		{
			desired_distance = configLoader.get<double>("Desired_distance");
			photo_distance = configLoader.get<double>("Photo_distance");
			std::cout << "Desired distance (real): " << desired_distance << std::endl;
		}

	std::cout << "Numeric locale active: " << setlocale(LC_NUMERIC, nullptr) << std::endl;

	std::filesystem::create_directories("logs");
	auto session_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count();
	orbit_log_path = "logs/orbit_" + std::to_string(session_ms) + ".csv";
	std::ofstream header(orbit_log_path);
	header << "t_ms,event,stage,waypoint_idx,shot_idx,x_b,y_b,dist_bump,"
		"angle_to_bump_deg,robot_heading_deg,start_bearing_deg,waypoint_bearing_deg,"
		"target_x,target_y,dist_waypoint,heading_error_deg,linear_v,angular_v,note\n";
	std::cout << "Orbit debug log: " << orbit_log_path << std::endl;

	// Same session id as the log file above, one subfolder per run so separate attempts don't
	// mix their photos together.
	photo_session_dir = (std::filesystem::path(photo_save_dir) / std::to_string(session_ms)).string();
}

void SpecificWorker::log_orbit(const std::string& csv_line)
{
	std::ofstream out(orbit_log_path, std::ios::app);
	if (out.is_open())
		out << csv_line << "\n";
}

std::string SpecificWorker::csv_row(const std::array<std::string, 19>& fields)
{
	// Exception messages (the usual source of "note" text) can contain embedded newlines and
	// commas, which would otherwise split into extra malformed rows or extra columns -- seen
	// live in logs/ before this existed.
	auto sanitize = [](std::string s)
	{
		for (char& c : s)
			if (c == ',') c = ';';
			else if (c == '\n' || c == '\r') c = ' ';
		return s;
	};

	std::ostringstream row;
	for (size_t i = 0; i < fields.size(); ++i)
	{
		if (i > 0)
			row << ",";
		row << sanitize(fields[i]);
	}
	return row.str();
}



void SpecificWorker::compute()
{
	auto_localization();
	// update_static_target_rt();

    if (check_affordance_active())
	{
		if (is_orbit_target()) // check for bump node
			orbit_target();
		else
		{
			reset_orbit_state();  // in case a previous, interrupted photo session left it mid-way
			follow_target(1.0f, 1.0f, desired_distance);
		}
	}
	else{
		stop_robot();
		was_following = false;  // next follow_target() call starts a fresh D-term baseline
		reset_orbit_state();
	}

	std::vector<float> actual_velocities = getVelocitiesFromDSR();

	if (has_significant_change(actual_velocities, last_velocities_readed)) {
		this->omnirobot_proxy->setSpeedBase(0.0, actual_velocities[0], actual_velocities[1]);
		std::cout << "setSpeedBase -> advx: " << actual_velocities[0] << " | rot: " << actual_velocities[1] << std::endl;
	}

	last_velocities_readed = actual_velocities;	
	

	update_or_create_imu_node();

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

#pragma region ROBOT_METHODS

void SpecificWorker::follow_target(float max_forward_speed_factor, float max_angular_speed_factor, float desired_distance)
{
    auto robot_node_opt = G->get_node("robot");
    if (!robot_node_opt.has_value())
    {
        std::cerr << "Robot node not found in DSR." << std::endl;
        return;
    }
    DSR::Node robot_node = robot_node_opt.value();

    auto target_edges = G->get_edges_by_type("TARGET");
    if (target_edges.empty())
    {
        std::cerr << "No target edges found in DSR." << std::endl;
        return;
    }
    DSR::Edge target_edge = target_edges[0];

    auto rt_edge_opt = rt->get_edge_RT(robot_node, target_edge.to());
    if (!rt_edge_opt.has_value())
    {
        std::cerr << "RT edge not found." << std::endl;
        return;
    }
    DSR::Edge rt_edge = rt_edge_opt.value();

    auto rt_translation_opt = G->get_attrib_by_name<rt_translation_att>(rt_edge);
    if (!rt_translation_opt.has_value())
    {
        std::cerr << "RT translation missing." << std::endl;
        return;
    }

    std::vector<float> t = rt_translation_opt.value();

    drive_to_local_point(t[0], t[1], desired_distance, max_forward_speed_factor, max_angular_speed_factor);
}

void SpecificWorker::drive_to_local_point(float x, float y, float desired_distance,
	float max_forward_speed_factor, float max_angular_speed_factor)
{
    auto robot_node_opt = G->get_node("robot");
    if (!robot_node_opt.has_value())
    {
        std::cerr << "Robot node not found in DSR." << std::endl;
        return;
    }
    DSR::Node robot_node = robot_node_opt.value();

    float distance_to_target = std::sqrt(x*x + y*y);
    float angle_to_target    = std::atan2(y, x);

    float distance_error = 0.0f;
    if (distance_to_target > 1e-3f)
        distance_error = (distance_to_target - desired_distance) / distance_to_target;

    if (distance_error < 0.0f)
        distance_error = 0.0f;

	float d_distance_error = 0.0f;
	float d_angle_error    = 0.0f;

    auto now = std::chrono::steady_clock::now();
	float dt = std::chrono::duration<float>(now - last_follow_time).count();
	last_follow_time = now;

	// Skip the derivative term on the first cycle after (re)starting to follow a target:
	// prev_distance_error/prev_angle_error/last_follow_time are otherwise stale (from
	// whatever target was last tracked, possibly a different mission), producing a bogus
	// error jump that briefly kicks linear_velocity/angular_velocity hard at start.
	if (dt > 1e-4f && was_following)
	{
		d_distance_error = (distance_error - prev_distance_error) / dt;
		float angle_diff = angle_to_target - prev_angle_error;
		angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));
		d_angle_error = angle_diff / dt;
		// d_angle_error    = (angle_to_target - prev_angle_error)   / dt;
	}

	prev_distance_error = distance_error;
	prev_angle_error    = angle_to_target;
	was_following = true;

    const float Kp_lin = 0.8f;
    const float Kd_lin = 0.1f;   

    const float Kp_ang = 1.0f;
    const float Kd_ang = 0.1f;   

	float linear_velocity  = Kp_lin * distance_error + Kd_lin * d_distance_error;
	float angular_velocity = Kp_ang * angle_to_target + Kd_ang * d_angle_error;

	float angle_attenuation = std::cos(std::clamp(angle_to_target, -HALF_PI, HALF_PI));
	linear_velocity *= angle_attenuation;

	linear_velocity = std::clamp(
		linear_velocity,
		0.0f,
		WEBOTS_MAX_LINEAR_SPEED * max_forward_speed_factor
	);

	angular_velocity = std::clamp(
		angular_velocity,
		-WEBOTS_MAX_ANGULAR_SPEED * max_angular_speed_factor,
		WEBOTS_MAX_ANGULAR_SPEED * max_angular_speed_factor
	);

    if (print_extra_info){
		auto ts_robot = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
	
		std::cout << "[" << ts_robot << "] RT target translation -> x: " << x << " | y: " << y
				  << " | angle_to_target: " << angle_to_target
				  << " | linear_v: " << linear_velocity
				  << " | angular_v: " << angular_velocity << std::endl;
	}    
	
	// std::cout << "Distance: "    << distance_to_target
        //          << "  Error: "     << distance_error
        //          << "  dError/dt: " << d_distance_error
        //          << "  Angle: "     << angle_to_target
        //          << "  Linear Vel: "<< linear_velocity
        //          << "  Angular Vel:"<< angular_velocity
	 	//		  << "RT target translation -> x: " << x << " | y: " << y 
		//		  << std::endl;

	G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node, linear_velocity);
	G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node, angular_velocity);
	G->update_node(robot_node);
}

void SpecificWorker::drive_straight(float distance_to_target, float desired_distance, float max_forward_speed_factor)
{
	auto robot_node_opt = G->get_node("robot");
	if (!robot_node_opt.has_value())
	{
		std::cerr << "Robot node not found in DSR." << std::endl;
		return;
	}
	DSR::Node robot_node = robot_node_opt.value();

	float distance_error = 0.0f;
	if (distance_to_target > 1e-3f)
		distance_error = (distance_to_target - desired_distance) / distance_to_target;
	if (distance_error < 0.0f)
		distance_error = 0.0f;

	const float Kp_lin = 0.8f;
	float linear_velocity = std::clamp(Kp_lin * distance_error, 0.0f, WEBOTS_MAX_LINEAR_SPEED * max_forward_speed_factor);

	G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node, linear_velocity);
	G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node, 0.0f);
	G->update_node(robot_node);
}

void SpecificWorker::update_static_target_rt()
{
	auto target_edges = G->get_edges_by_type("TARGET");
	if (target_edges.empty())
		return;

	auto target_node_opt = G->get_node(target_edges[0].to());
	if (!target_node_opt.has_value())
		return;
	DSR::Node target_node = target_node_opt.value();

	auto pos_it = target_node.attrs().find("problem_position");
	if (pos_it == target_node.attrs().end())
		return;  // Not a static-position target (e.g. "person"): its RT is maintained elsewhere.

	auto* pos_mm = std::get_if<std::vector<float>>(&pos_it->second.value());
	if (!pos_mm || pos_mm->size() < 3)
		return;

	auto robot_node_opt = G->get_node("robot");
	auto root_node_opt = G->get_node("root");
	if (!robot_node_opt.has_value() || !root_node_opt.has_value())
		return;

	auto root_robot_rt_opt = rt->get_edge_RT(root_node_opt.value(), robot_node_opt.value().id());
	if (!root_robot_rt_opt.has_value())
		return;
	DSR::Edge root_robot_rt = root_robot_rt_opt.value();

	auto t_rr_opt = G->get_attrib_by_name<rt_translation_att>(root_robot_rt);
	auto q_rr_opt = G->get_attrib_by_name<rt_quaternion_att>(root_robot_rt);
	if (!t_rr_opt.has_value() || !q_rr_opt.has_value())
		return;

	std::vector<float> t_rr = t_rr_opt.value();
	std::vector<float> q_rr = q_rr_opt.value();

	Eigen::Vector3f root_robot_t(t_rr[0], t_rr[1], t_rr[2]);
	Eigen::Quaternionf root_robot_q(q_rr[3], q_rr[0], q_rr[1], q_rr[2]);  // stored as [x,y,z,w]

	// problem_position is mm (project-wide convention); root->robot here is meters
	// (concept_robot's own convention, see mm_m_unit_mismatch).
	Eigen::Vector3f root_target_t((*pos_mm)[0] / 1000.f, (*pos_mm)[1] / 1000.f, (*pos_mm)[2] / 1000.f);
	// Eigen::Vector3f root_target_t((*pos_mm)[0], (*pos_mm)[1], (*pos_mm)[2]);

	Eigen::Vector3f local_t = root_robot_q.inverse() * (root_target_t - root_robot_t);

	rt->insert_or_assign_edge_RT(robot_node_opt.value(), target_node.id(),
		{local_t.x(), local_t.y(), local_t.z()},
		{0.f, 0.f, 0.f});
}

std::vector<float> SpecificWorker::auto_localization()
{
	std::vector<float> robot_pose = {0,0,0,0,0,0,1}; // {x, y, z, qx, qy, qz, qw}
	if (simulated){
		auto webots_pose = this->webots2robocomp_proxy->getObjectPose(robot_DEF);
		robot_pose[0] = webots_pose.position.y / 1000.f;
		robot_pose[1] = webots_pose.position.x / 1000.f;
		robot_pose[2] = webots_pose.position.z / 1000.f;
		Eigen::Quaternionf quat(webots_pose.orientation.w, webots_pose.orientation.x, webots_pose.orientation.y, webots_pose.orientation.z);
		quat.normalize();
		robot_pose[3] = quat.x();
		robot_pose[4] = quat.y();
		robot_pose[5] = quat.z();
		robot_pose[6] = quat.w();
	}
	else{
		if (last_odometry.empty())
			return robot_pose;
			
		robot_pose[0] = last_odometry[0];
		robot_pose[1] = last_odometry[1];
		robot_pose[2] = 0.0f;
		Eigen::AngleAxisf rot_z(last_odometry[2], Eigen::Vector3f::UnitZ());
		Eigen::Quaternionf quat(rot_z);
		robot_pose[3] = quat.x();
		robot_pose[4] = quat.y();
		robot_pose[5] = quat.z();
		robot_pose[6] = quat.w();
	}

	if (!has_significant_change(robot_pose, last_robot_pose))
	{
		last_robot_pose = robot_pose;
		return robot_pose;
	}

	auto robot_node_opt = G->get_node("robot");
    auto root_node_opt = G->get_node("root");

	DSR::Node root_node, robot_node;

	if (!robot_node_opt.has_value())
	{
		std::cerr << "Robot node not found in DSR . Creating new robot node." << std::endl;
		robot_node = DSR::Node::create<robot_node_type>("robot");
		G->insert_node(robot_node);
	}
	else 
		robot_node = robot_node_opt.value();

	if (!root_node_opt.has_value())
	{
		std::cerr << "Root node not found in DSR. Creating new root node." << std::endl;
		root_node = DSR::Node::create<root_node_type>("root");
		G->insert_node(root_node);
	}
	else
		root_node = root_node_opt.value();

	auto rt_edge_opt = rt->get_edge_RT(root_node, robot_node.id());
	if (!rt_edge_opt.has_value())
	{
		std::cerr << "RT edge between root and robot not found. Creating new RT edge." << std::endl;
		DSR::Edge new_rt_edge;
		new_rt_edge.from(root_node.id());
		new_rt_edge.to(robot_node.id());
		new_rt_edge.type("RT");
		G->add_or_modify_attrib_local<rt_translation_att>(new_rt_edge, (std::vector<float>){robot_pose[0], robot_pose[1], robot_pose[2]});
		G->add_or_modify_attrib_local<rt_quaternion_att>(new_rt_edge, (std::vector<float>){robot_pose[3], robot_pose[4], robot_pose[5], robot_pose[6]});
		G->insert_or_assign_edge(new_rt_edge);
	}
	else
	{
		DSR::Edge rt_edge = rt_edge_opt.value();
		G->add_or_modify_attrib_local<rt_translation_att>(rt_edge, (std::vector<float>){robot_pose[0], robot_pose[1], robot_pose[2]});
		G->add_or_modify_attrib_local<rt_quaternion_att>(rt_edge, (std::vector<float>){robot_pose[3], robot_pose[4], robot_pose[5], robot_pose[6]});
		G->insert_or_assign_edge(rt_edge);
	}

	return robot_pose;
}

#pragma endregion ROBOT_METHODS

#pragma region DSR

std::vector<float> SpecificWorker::getVelocitiesFromDSR()
{
	std::vector<float> velocities = {0.0, 0.0}; // {advx, rot}

	//Access to DSR to get the desired velocities
	auto optional_robot_node = G->get_node("robot");
	if (optional_robot_node.has_value())
	{
		auto robot_node = optional_robot_node.value();
		auto optional_adv_speed = G->get_attrib_by_name<robot_ref_adv_speed_att>(robot_node.id());
		if (optional_adv_speed.has_value())
		{
			velocities[0] = optional_adv_speed.value() * 1000;
		}
		auto optional_rot_speed = G->get_attrib_by_name<robot_ref_rot_speed_att>(robot_node.id());
		if (optional_rot_speed.has_value())
		{
			velocities[1] = optional_rot_speed.value();
		}
	}

	return velocities;
}

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


void SpecificWorker::update_or_create_imu_node()
{
	std::vector<float> acceleration, angularVel;

	try{
		auto acceleration_raw = this->imu_proxy->getAcceleration();
		auto angularVel_raw = this->imu_proxy->getAngularVel();

		acceleration = {acceleration_raw.XAcc, acceleration_raw.YAcc, acceleration_raw.ZAcc};
		angularVel = {angularVel_raw.XGyr, angularVel_raw.YGyr, angularVel_raw.ZGyr};

	}catch(const Ice::Exception& ex)
    {
        std::cout <<"IMU proxy exception:"<< ex << std::endl;
        throw;
    }

	if (acceleration.empty() || angularVel.empty())
		return;	

	if (auto imu_node_opt = G->get_node("imu"); imu_node_opt.has_value())
	{
		auto imu_real_node = imu_node_opt.value();
		if(has_significant_change(last_acceleration_measurement, acceleration) 
		or has_significant_change(last_angular_velocity_measurement, angularVel)){
			G->add_or_modify_attrib_local<imu_accelerometer_att>(imu_real_node, acceleration);
			G->add_or_modify_attrib_local<imu_gyroscope_att>(imu_real_node, angularVel);
			G->update_node(imu_real_node);

			last_acceleration_measurement = acceleration;
			last_angular_velocity_measurement = angularVel;
		}
	}
	else
	{
		auto robot_node_opt = G->get_node("robot");
		if (!robot_node_opt.has_value())
		{
			std::cerr << "Robot node not found in DSR. Cannot create IMU node without robot node." << std::endl;
			return;
		}
		DSR::Node robot_node = robot_node_opt.value();
		
		std::cout << "Creating IMU node in DSR." << std::endl; 
		DSR::Node imu_node = DSR::Node::create<imu_node_type>("imu");
		auto pos_x = G->get_attrib_by_name<pos_x_att>(robot_node.id()).value();
		auto pos_y = G->get_attrib_by_name<pos_y_att>(robot_node.id()).value();
		auto level = G->get_attrib_by_name<level_att>(robot_node.id()).value();
		G->add_or_modify_attrib_local<parent_att>(imu_node, robot_node.id());
		G->add_or_modify_attrib_local<pos_x_att>(imu_node, pos_x+100);
		G->add_or_modify_attrib_local<pos_y_att>(imu_node, pos_y+100);
		G->add_or_modify_attrib_local<level_att>(imu_node, level+1);

		G->add_or_modify_attrib_local<imu_accelerometer_att>(imu_node, acceleration);
		G->add_or_modify_attrib_local<imu_gyroscope_att>(imu_node, angularVel);

		G->insert_node(imu_node);
		G->update_node(imu_node);
	
		DSR::Edge imu_edge;
		imu_edge.from(robot_node.id());
		imu_edge.to(imu_node.id());
		imu_edge.type("has");
		G->insert_or_assign_edge(imu_edge);
	
	}
}

// Shared by check_affordance_active() and complete_photo_affordance(): the affordance node
// reached from the current TARGET via TARGET->has_intention.
static std::optional<DSR::Node> get_target_affordance_node(DSR::DSRGraph* G)
{
	auto target_edges = G->get_edges_by_type("TARGET");
	auto has_intention_edges = G->get_edges_by_type("has_intention");
	for (const auto& target_edge : target_edges)
		for (const auto& intention_edge : has_intention_edges)
			if (intention_edge.from() == target_edge.to())
				return G->get_node(intention_edge.to());
	return std::nullopt;
}

bool SpecificWorker::check_affordance_active()
{
	auto affordance_node_opt = get_target_affordance_node(G.get());
	if (!affordance_node_opt.has_value())
		return false;
	return G->get_attrib_by_name<aff_interacting_att>(affordance_node_opt.value().id()).value();
}

void SpecificWorker::stop_robot()
{
	auto robot_node_opt = G->get_node("robot");
	if (!robot_node_opt.has_value())	{
		std::cerr << "Robot node not found in DSR. Cannot stop robot without robot node." << std::endl;
		return;
	}

	auto DSR_velocities = getVelocitiesFromDSR();
	if (DSR_velocities[0] == 0.0 && DSR_velocities[1] == 0.0){
		return; // Robot is already stopped in DSR, no need to update.
	}

	DSR::Node robot_node = robot_node_opt.value();
	G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node, (float)0.0);
	G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node, (float)0.0);
	G->update_node(robot_node);
}

bool SpecificWorker::is_orbit_target()
{
	auto target_edges = G->get_edges_by_type("TARGET");
	if (target_edges.empty())
		return false;
	auto target_node_opt = G->get_node(target_edges[0].to());
	return target_node_opt.has_value() && target_node_opt.value().name() == "bump";
}

void SpecificWorker::reset_orbit_state()
{
	orbit_stage = OrbitStage::TRAVEL;
	orbit_waypoint_idx = 0;
	orbit_shot_idx = 0;
	orbit_bearing_captured = false;
	travel_is_rotating = true;
}

void SpecificWorker::rotate_in_place(float heading_error, float max_angular_speed_factor)
{
	auto robot_node_opt = G->get_node("robot");
	if (!robot_node_opt.has_value())
	{
		std::cerr << "Robot node not found in DSR." << std::endl;
		return;
	}
	DSR::Node robot_node = robot_node_opt.value();

	const float Kp_ang = 1.0f;
	float angular_velocity = std::clamp(
		Kp_ang * heading_error,
		-WEBOTS_MAX_ANGULAR_SPEED * max_angular_speed_factor,
		WEBOTS_MAX_ANGULAR_SPEED * max_angular_speed_factor
	);

	G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node, 0.0f);
	G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node, angular_velocity);
	G->update_node(robot_node);
}

void SpecificWorker::take_photo(const std::string& label, float distance_to_bump)
{
	auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
	auto log_result = [&](const std::string& outcome, const std::string& path_or_reason)
	{
		log_orbit(csv_row({std::to_string(now_ms), "photo", "", "", "", "", "",
			std::to_string(distance_to_bump), "", "", "", "", "", "", "", "", "", "",
			label + " " + outcome + ": " + path_or_reason}));
	};

	try
	{
		// Camera360RGB looked like the natural fit (concept_robot's own interface, no cross-agent
		// hop) but webots-bridge has it permanently disabled (pars.camera360 defaults false and
		// nothing ever sets it true) -- every getROI() call throws. CameraRGBDSimple is the same
		// interface vision_sam already captures through successfully; only .image is used here
		// (DINOv3 doesn't need depth), so RGB-only intent is preserved.
		auto img = camerargbdsimple_proxy->getImage("camera");
		if (img.image.empty() || img.width <= 0 || img.height <= 0)
		{
			std::cerr << "orbit_target: empty image from Camera360RGB, skipping capture." << std::endl;
			log_result("FAILED", "empty image from Camera360RGB");
			return;
		}

		std::filesystem::path dir = std::filesystem::path(photo_session_dir) / label;
		std::filesystem::create_directories(dir);

		auto file_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
		std::filesystem::path filepath = dir /
			(label + "_" + std::to_string(file_ms) + "_" + std::to_string(photo_counter++) + ".jpg");

		// img.image is RGB (webots-bridge converts RGBA->RGB before sending); OpenCV's
		// imwrite/imencode assume BGR for standard formats, so swap channels before saving.
		cv::Mat rgb(img.height, img.width, CV_8UC3, const_cast<unsigned char*>(img.image.data()));
		cv::Mat bgr;
		cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
		if (!cv::imwrite(filepath.string(), bgr))
		{
			std::cerr << "orbit_target: cv::imwrite failed for " << filepath.string() << std::endl;
			log_result("FAILED", "cv::imwrite failed");
			return;
		}

		std::cout << "orbit_target: saved " << filepath.string()
			<< " (dist_to_bump=" << distance_to_bump << "m)" << std::endl;
		log_result("OK", std::filesystem::absolute(filepath).string());
	}
	catch (const std::exception& e)
	{
		std::cerr << "orbit_target: take_photo failed: " << e.what() << std::endl;
		log_result("FAILED", e.what());
	}
}

void SpecificWorker::complete_photo_affordance()
{
	auto affordance_node_opt = get_target_affordance_node(G.get());
	if (!affordance_node_opt.has_value())
		return;
	DSR::Node affordance_node = affordance_node_opt.value();
	G->add_or_modify_attrib_local<aff_interacting_att>(affordance_node, false);
	G->update_node(affordance_node);
}

const char* SpecificWorker::orbit_stage_name(OrbitStage s)
{
	switch (s)
	{
		case OrbitStage::TRAVEL: return "TRAVEL";
		case OrbitStage::SHOOT_CON: return "SHOOT_CON";
		case OrbitStage::TURN_AWAY: return "TURN_AWAY";
		case OrbitStage::SHOOT_SIN: return "SHOOT_SIN";
		case OrbitStage::NEXT_WAYPOINT: return "NEXT_WAYPOINT";
		case OrbitStage::DONE: return "DONE";
	}
	return "?";
}

void SpecificWorker::orbit_target()
{
	auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();

	auto robot_node_opt = G->get_node("robot");
	auto root_node_opt  = G->get_node("root");
	auto target_edges = G->get_edges_by_type("TARGET");
	if (!robot_node_opt.has_value() || !root_node_opt.has_value() || target_edges.empty())
	{
		log_orbit(csv_row({std::to_string(now_ms), "bail", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "missing robot/root node or TARGET edge"}));
		return;
	}
	DSR::Node robot_node = robot_node_opt.value();

	// Live robot->bump local position, published every cycle by concept_bump (same read as
	// follow_target(), just kept as x,y instead of handed to the P/D controller directly).
	auto bump_rt_opt = rt->get_edge_RT(robot_node, target_edges[0].to());
	if (!bump_rt_opt.has_value())
	{
		log_orbit(csv_row({std::to_string(now_ms), "bail", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "robot->bump RT edge not found (concept_bump not publishing?)"}));
		return;
	}
	auto t_rb_opt = G->get_attrib_by_name<rt_translation_att>(bump_rt_opt.value());
	if (!t_rb_opt.has_value())
	{
		log_orbit(csv_row({std::to_string(now_ms), "bail", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "robot->bump RT has no translation attribute"}));
		return;
	}
	std::vector<float> t_rb = t_rb_opt.value();
	float x_b = t_rb[0];
	float y_b = t_rb[1];

	// Robot's absolute heading, from root->robot RT (same formula concept_bump uses).
	auto root_robot_rt_opt = rt->get_edge_RT(root_node_opt.value(), robot_node.id());
	if (!root_robot_rt_opt.has_value())
	{
		log_orbit(csv_row({std::to_string(now_ms), "bail", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "root->robot RT edge not found"}));
		return;
	}
	auto q_rr_opt = G->get_attrib_by_name<rt_quaternion_att>(root_robot_rt_opt.value());
	if (!q_rr_opt.has_value())
	{
		log_orbit(csv_row({std::to_string(now_ms), "bail", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "root->robot RT has no quaternion attribute"}));
		return;
	}
	std::vector<float> q_rr = q_rr_opt.value();
	Eigen::Quaternionf root_robot_q(q_rr[3], q_rr[0], q_rr[1], q_rr[2]);
	root_robot_q.normalize();
	float robot_heading = std::atan2(
		2.f * (root_robot_q.w() * root_robot_q.z() + root_robot_q.x() * root_robot_q.y()),
		1.f - 2.f * (root_robot_q.y() * root_robot_q.y() + root_robot_q.z() * root_robot_q.z()));

	const float PI = std::numbers::pi_v<float>;
	auto wrap = [PI](float a) { return std::atan2(std::sin(a), std::cos(a)); };

	float angle_to_bump = std::atan2(y_b, x_b);  // heading_error to face the bump right now

	if (!orbit_bearing_captured)
	{
		// World-ish bearing from the bump to the robot's current position: the reference the k
		// waypoints are spaced around, so waypoint 0 is wherever the robot already is (minimal
		// extra travel after the approach).
		// The +PI/2 term is NOT optional: concept_bump's world->local formula (which x_b,y_b
		// come from) is a rotation by -(heading+90 deg), not by -heading as the textbook
		// world_to_local would be (verified by inverting that formula by hand), so recovering a
		// world bearing from a local one needs the same +90 deg correction.
		orbit_start_bearing = wrap(robot_heading + std::atan2(-y_b, -x_b) + PI / 2.0f);
		orbit_bearing_captured = true;
	}

	float log_waypoint_bearing = std::numeric_limits<float>::quiet_NaN();
	float log_target_x = std::numeric_limits<float>::quiet_NaN();
	float log_target_y = std::numeric_limits<float>::quiet_NaN();
	float log_heading_error = std::numeric_limits<float>::quiet_NaN();

	switch (orbit_stage)
	{
		case OrbitStage::TRAVEL:
		{
			int k = std::max(1, orbit_points_k);
			float waypoint_bearing = wrap(orbit_start_bearing + 2.f * PI * orbit_waypoint_idx / k);
			log_waypoint_bearing = waypoint_bearing;

			// Waypoint = bump_world + photo_distance * (cos,sin)(waypoint_bearing); expressed
			// directly in the robot's current local frame as local(bump) + world_to_local(offset)
			// -- world_to_local must be the same formula concept_bump uses for its own
			// world->local projection (get_bump_relative_position()), NOT the textbook rotation:
			// this project's quaternion/axis convention isn't the standard one, and that formula
			// is the one actually validated live (robot converges correctly onto the bump).
			float dx_world = photo_distance * std::cos(waypoint_bearing);
			float dy_world = photo_distance * std::sin(waypoint_bearing);
			float local_dx = -std::sin(robot_heading) * dx_world + std::cos(robot_heading) * dy_world;
			float local_dy = -std::cos(robot_heading) * dx_world - std::sin(robot_heading) * dy_world;
			float target_x = x_b + local_dx;
			float target_y = y_b + local_dy;
			log_target_x = target_x;
			log_target_y = target_y;

			float dist_to_waypoint = std::sqrt(target_x * target_x + target_y * target_y);
			float angle_to_waypoint = std::atan2(target_y, target_x);
			log_heading_error = angle_to_waypoint;

			// Deliberately simple for now (easy to replace with the combined drive later): look
			// at the waypoint first (pure rotation), then drive straight at it (pure
			// translation, no steering correction). Never combines both, so it can't reproduce
			// the earlier combined-motion instability.
			//
			// Hysteresis on the rotate/drive switch (two thresholds, not one): a single shared
			// tolerance made this chatter every 20-300ms once heading drifted slightly during a
			// drive leg (confirmed in logs/), since the smallest drift past the tolerance flips
			// straight back to rotating. Widening the "start rotating again" threshold well past
			// the "good enough, start driving" one gives each phase room to run to completion
			// instead of re-deciding from scratch every cycle.
			if (dist_to_waypoint < ORBIT_ARRIVAL_TOLERANCE)
			{
				stop_robot();
				orbit_stage = OrbitStage::SHOOT_CON;
				orbit_shot_idx = 0;
			}
			else
			{
				if (travel_is_rotating && std::abs(angle_to_waypoint) < ORBIT_HEADING_TOLERANCE)
					travel_is_rotating = false;
				else if (!travel_is_rotating && std::abs(angle_to_waypoint) > TRAVEL_ROTATE_REENTRY_TOLERANCE)
					travel_is_rotating = true;

				if (travel_is_rotating)
					rotate_in_place(angle_to_waypoint);
				else
					drive_straight(dist_to_waypoint, ORBIT_ARRIVAL_TOLERANCE, 1.0f);
			}
			break;
		}

		case OrbitStage::SHOOT_CON:
		case OrbitStage::SHOOT_SIN:
		{
			bool is_con = (orbit_stage == OrbitStage::SHOOT_CON);
			float center = is_con ? 0.0f : PI;  // heading_error=0 means "facing the bump"
			int X = std::max(1, photos_per_point);
			float offset = (float(orbit_shot_idx) - (X - 1) / 2.0f) * photo_angular_step;
			// Same error convention as drive_to_local_point()/follow_target() (angular_velocity =
			// Kp * error, no negation), just generalized from an implicit target of 0 to "center+offset".
			float heading_error = wrap(angle_to_bump - (center + offset));
			log_heading_error = heading_error;

			if (std::abs(heading_error) < ORBIT_HEADING_TOLERANCE)
			{
				stop_robot();
				take_photo(is_con ? "con_bache" : "sin_bache", std::sqrt(x_b * x_b + y_b * y_b));
				orbit_shot_idx++;
				if (orbit_shot_idx >= X)
					orbit_stage = is_con ? OrbitStage::TURN_AWAY : OrbitStage::NEXT_WAYPOINT;
			}
			else
				rotate_in_place(heading_error);
			break;
		}

		case OrbitStage::TURN_AWAY:
		{
			// Same rotate_in_place() primitive as the fine panning above, just aimed at a
			// farther target (~180 deg): never a single raw "rotate 180" command, always the
			// same P-controlled, velocity-clamped step.
			float heading_error = wrap(angle_to_bump - PI);
			log_heading_error = heading_error;
			if (std::abs(heading_error) < ORBIT_HEADING_TOLERANCE)
			{
				orbit_stage = OrbitStage::SHOOT_SIN;
				orbit_shot_idx = 0;
			}
			else
				rotate_in_place(heading_error);
			break;
		}

		case OrbitStage::NEXT_WAYPOINT:
		{
			orbit_waypoint_idx++;
			orbit_stage = (orbit_waypoint_idx >= std::max(1, orbit_points_k))
				? OrbitStage::DONE : OrbitStage::TRAVEL;
			travel_is_rotating = true;  // look at the new waypoint before driving to it
			break;
		}

		case OrbitStage::DONE:
		{
			stop_robot();
			complete_photo_affordance();
			break;
		}
	}

	std::vector<float> velocities = getVelocitiesFromDSR();
	float dist_bump = std::sqrt(x_b * x_b + y_b * y_b);
	float dist_waypoint = std::isnan(log_target_x) ? std::numeric_limits<float>::quiet_NaN()
		: std::sqrt(log_target_x * log_target_x + log_target_y * log_target_y);

	auto rad2deg = [](float r) { return std::isnan(r) ? r : r * 180.f / std::numbers::pi_v<float>; };
	auto fmt = [](float v) { return std::isnan(v) ? std::string() : std::to_string(v); };

	log_orbit(csv_row({
		std::to_string(now_ms), "cycle", orbit_stage_name(orbit_stage),
		std::to_string(orbit_waypoint_idx), std::to_string(orbit_shot_idx),
		fmt(x_b), fmt(y_b), fmt(dist_bump),
		fmt(rad2deg(angle_to_bump)), fmt(rad2deg(robot_heading)),
		fmt(rad2deg(orbit_start_bearing)), fmt(rad2deg(log_waypoint_bearing)),
		fmt(log_target_x), fmt(log_target_y), fmt(dist_waypoint),
		fmt(rad2deg(log_heading_error)),
		fmt(velocities[0] / 1000.f), fmt(velocities[1]), ""
	}));
}

#pragma endregion DSR

//SUBSCRIPTION to newFullPose method from FullPoseEstimationPub interface
// void SpecificWorker::FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)
// {
// 	if (simulated)
// 		return;

//     if (!std::isfinite(pose.y) || !std::isfinite(pose.rz))
//     {
//         std::cerr << "[FullPoseEstimationPub_newFullPose] WARNING: invalid pose received "
//                   << "(y=" << pose.y << ", rz=" << pose.rz << "), skipping." << std::endl;
//         return;
//     }

// 	float adv, last_x, last_y, last_theta;
// 	adv = pose.y;

// 	if (last_odometry.empty())
// 	{
// 		last_x = 0;
// 		last_y = 0;
// 		last_theta = 0;
// 	}
// 	else
// 	{		
// 		last_x = last_odometry[0];
// 		last_y = last_odometry[1];
// 		last_theta = last_odometry[2];
// 	}

// 		float theta = last_theta + pose.rz;
// 		float x = adv * std::cos(theta) + last_x;
// 		float y = adv * std::sin(theta) + last_y;

// 		last_odometry = {x, y, theta};

// 		if (print_extra_info){
// 			std::cout << "[FullPoseEstimationPub_newFullPose]" << std::endl;
// 			std::cout << "  Raw pose    -> y (adv): " << adv 
// 					<< " | rz: " << pose.rz << std::endl;
// 			std::cout << "  Last odom   -> x: " << last_x 
// 					<< " | y: " << last_y 
// 					<< " | theta: " << last_theta << std::endl;
// 			std::cout << "  New odom    -> x: " << x 
// 					<< " | y: " << y 
// 					<< " | theta: " << theta << std::endl;
// 		}
// }

void SpecificWorker::FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)
{
    if (simulated)
        return;

    if (!std::isfinite(pose.vy) || !std::isfinite(pose.vx) || !std::isfinite(pose.vrz))
    {
        std::cerr << "[FullPoseEstimationPub_newFullPose] WARNING: invalid pose, skipping." << std::endl;
        return;
    }

    // Calcular dt a partir del timestamp de la pose (viene en ms)
    if (last_timestamp == 0)
    {
        last_timestamp = pose.timestamp;
        last_odometry = {0.f, 0.f, 0.f};
        return;
    }

    float dt = (pose.timestamp - last_timestamp) / 1000.f;
    last_timestamp = pose.timestamp;

    if (dt <= 0.f || dt > 1.f)
    {
        std::cerr << "[FullPoseEstimationPub_newFullPose] WARNING: anomalous dt=" << dt << ", skipping." << std::endl;
        return;
    }

    // Ventana deslizante de velocidades
    velocity_window.push_back({pose.vx, pose.vy, pose.vrz});
    if (velocity_window.size() > ODOMETRY_WINDOW_SIZE)
        velocity_window.pop_front();

    if (velocity_window.size() < ODOMETRY_WINDOW_SIZE)
        return;

    float avg_vx = 0.f, avg_vy = 0.f, avg_vrz = 0.f;
    for (const auto& [vx, vy, vrz] : velocity_window)
    {
        avg_vx  += vx;
        avg_vy  += vy;
        avg_vrz += vrz;
    }
    avg_vx  /= ODOMETRY_WINDOW_SIZE;
    avg_vy  /= ODOMETRY_WINDOW_SIZE;
    avg_vrz /= ODOMETRY_WINDOW_SIZE;

	if (std::abs(avg_vy) < LINEAR_VELOCITY_DEADBAND) avg_vy = 0.f;
	if (std::abs(avg_vx) < LINEAR_VELOCITY_DEADBAND) avg_vx = 0.f;
	if (std::abs(avg_vrz) < ANGULAR_VELOCITY_DEADBAND) avg_vrz = 0.f;

    float last_x     = last_odometry[0];
    float last_y     = last_odometry[1];
    float last_theta = last_odometry[2];

	float theta = last_theta + avg_vrz * dt;
	float x     = last_x + (avg_vy * std::sin(last_theta) + avg_vx * std::cos(last_theta)) * dt;
	float y     = last_y + (avg_vy * std::cos(last_theta) - avg_vx * std::sin(last_theta)) * dt;

    last_odometry = {x, y, theta};

    if (print_extra_info)
    {
        std::cout << "[FullPoseEstimationPub_newFullPose]\n"
                  << "  Raw vel    -> vy: " << pose.vy  << " | vx: " << pose.vx  << " | vrz: " << pose.vrz << "\n"
                  << "  Avg vel    -> vy: " << avg_vy   << " | vx: " << avg_vx   << " | vrz: " << avg_vrz  << "\n"
                  << "  dt         -> " << dt << " s\n"
                  << "  Last odom  -> x: " << last_x    << " | y: " << last_y    << " | theta: " << last_theta << "\n"
                  << "  New odom   -> x: " << x         << " | y: " << y         << " | theta: " << theta << std::endl;
    }
}



/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// RoboCompCameraRGBDSimple::TRGBD this->camerargbdsimple_proxy->getAll(string camera)
// RoboCompCameraRGBDSimple::TDepth this->camerargbdsimple_proxy->getDepth(string camera)
// RoboCompCameraRGBDSimple::TImage this->camerargbdsimple_proxy->getImage(string camera)
// RoboCompCameraRGBDSimple::TPoints this->camerargbdsimple_proxy->getPoints(string camera)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::Point3D
// RoboCompCameraRGBDSimple::TPoints
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompIMU you can call this methods:
// RoboCompIMU::Acceleration this->imu_proxy->getAcceleration()
// RoboCompIMU::Gyroscope this->imu_proxy->getAngularVel()
// RoboCompIMU::DataImu this->imu_proxy->getDataImu()
// RoboCompIMU::Magnetic this->imu_proxy->getMagneticFields()
// RoboCompIMU::Orientation this->imu_proxy->getOrientation()
// RoboCompIMU::void this->imu_proxy->resetImu()

/**************************************/
// From the RoboCompIMU you can use this types:
// RoboCompIMU::Acceleration
// RoboCompIMU::Gyroscope
// RoboCompIMU::Magnetic
// RoboCompIMU::Orientation
// RoboCompIMU::DataImu

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
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

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

