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

    //initializeCODE

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

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
		auto bottle_pose = detect_bottle();
		if (bottle_pose.z < 300 or hasFallen(bottle_pose))
		{
			std::cout << "The bottle is related to the robot and has fallen." << std::endl;
			change_rt_from_robot_to_root();
		}
		else
		{
			std::cout << "The bottle is related to the robot and is upright." << std::endl;
		}
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

void SpecificWorker::change_rt_from_robot_to_root()
{
	G->delete_edge("robot","bottle","RT");

	auto root_node = G->get_node("root").value();
	auto bottle_node = G->get_node("bottle").value();

	DSR::Edge loss_edge;
	loss_edge.from(root_node.id());
	loss_edge.to(bottle_node.id());
	loss_edge.type("RT");
	
	G->insert_or_assign_edge(loss_edge);
}


/**************************************/
/********* DETECTION METHODS **********/

SpecificWorker::BottlePose SpecificWorker::detect_bottle()
{	
	RoboCompWebots2Robocomp::ObjectPose bottle_pose;

	if (simulated)
	{	
		std::cout << "Detecting bottle in simulation..." << std::endl;
		bottle_pose = this->webots2robocomp_proxy->getObjectPose("flacon"); //MILLIMETERS
	}
	else
	{
		// TODO: Implement real detection method here and fill the bottle_pose variable with the obtained values.
		// Example of filling the bottle_pose variable with dummy values:
		bottle_pose.position.x = 100.0; // Replace with actual x position
		bottle_pose.position.y = 200.0; // Replace with actual y position
		bottle_pose.position.z = 300.0; // Replace with actual z position
		bottle_pose.orientation.x = 0.0; // Replace with actual qx orientation
		bottle_pose.orientation.y = 0.0; // Replace with actual qy orientation
		bottle_pose.orientation.z = 0.0; // Replace with actual qz orientation
		bottle_pose.orientation.w = 1.0; // Replace with actual qw orientation
	}

	SpecificWorker::BottlePose pose;
	pose.x = bottle_pose.position.x;
	pose.y = bottle_pose.position.y;
	pose.z = bottle_pose.position.z;
	pose.qx = bottle_pose.orientation.x;
	pose.qy = bottle_pose.orientation.y;
	pose.qz = bottle_pose.orientation.z;
	pose.qw = bottle_pose.orientation.w;

	return pose;
}


//SUBSCRIPTION to setVisualObjects method from VisualElementsPub interface
void SpecificWorker::VisualElementsPub_setVisualObjects(RoboCompVisualElementsPub::TData data)
{
//subscribesToCODE

}



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

