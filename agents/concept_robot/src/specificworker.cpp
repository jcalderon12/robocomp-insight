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
	qInstallMessageHandler([](QtMsgType, const QMessageLogContext&, const QString&) {});

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

	graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
	//graph_viewer->add_custom_widget_to_dock("CustomWidget", &custom_widget);

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

	rt = G->get_rt_api();

	last_velocities_readed = getVelocitiesFromDSR();
}



void SpecificWorker::compute()
{
    if (queck_affordance_active())
	{
		follow_target(0.3f, 0.5f, 1.2f);
	}
	else{
		stop_robot();
	}

	std::vector<float> actual_velocities = getVelocitiesFromDSR();

	if (has_significant_change(actual_velocities, last_velocities_readed))
		this->omnirobot_proxy->setSpeedBase(0.0, actual_velocities[0], -actual_velocities[1]);

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

#pragma region ROBOT_CONTROL

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

    float x = t[0];   
    float y = t[1];   

    float distance_to_target = std::sqrt(x*x + y*y);
    float angle_to_target = std::atan2(y, x);

	float distance_error = distance_to_target - desired_distance;

    if (distance_error < 0.0f)
        distance_error = 0.0f;

    float Kp_lin = 1.0f;
    float Kp_ang = 2.0f;

    float linear_velocity  = Kp_lin * distance_error;
    float angular_velocity = Kp_ang * angle_to_target;

    if (std::abs(angle_to_target) > 0.4f)
        linear_velocity *= 0.3f;

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

	if (print_extra_info)
		std::cout << "Distance: " << distance_to_target
				<< "  Error: " << distance_error
				<< "  Angle: " << angle_to_target
				<< "  Linear Vel: " << linear_velocity
				<< "  Angular Vel: " << angular_velocity << std::endl;


    G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node, linear_velocity);
    G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node, angular_velocity);
    G->update_node(robot_node);
}



#pragma endregion ROBOT_CONTROL

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

	auto acceleration_raw = this->imu_proxy->getAcceleration();
	auto angularVel_raw = this->imu_proxy->getAngularVel();
	std::vector<float> acceleration = {acceleration_raw.XAcc, acceleration_raw.YAcc, acceleration_raw.ZAcc};
	std::vector<float> angularVel = {angularVel_raw.XGyr, angularVel_raw.YGyr, angularVel_raw.ZGyr};

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

bool SpecificWorker::queck_affordance_active()
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

#pragma endregion DSR

//SUBSCRIPTION to newFullPose method from FullPoseEstimationPub interface
void SpecificWorker::FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)
{
//subscribesToCODE

}



/**************************************/
// From the RoboCompCamera360RGB you can call this methods:
// RoboCompCamera360RGB::TImage this->camera360rgb_proxy->getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

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
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->resetWebots()
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setDoorAngle(float angle)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setPathToHuman(int humanId, RoboCompGridder::TPath path)

/**************************************/
// From the RoboCompWebots2Robocomp you can use this types:
// RoboCompWebots2Robocomp::Vector3
// RoboCompWebots2Robocomp::Quaternion

