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
		

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}

	rt = G->get_rt_api();
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

}



void SpecificWorker::compute()
{
	auto distance_to_person = get_distance_to_person();
	update_distance_to_person(distance_to_person);

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
			auto state = follow_person(distance_to_person);
			if (state == false){
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

std::vector<float> SpecificWorker::get_distance_to_person()
{
	RoboCompWebots2Robocomp::ObjectPose person_pose = this->webots2robocomp_proxy->getObjectPose(person_def);
	RoboCompWebots2Robocomp::ObjectPose robot_pose = this->webots2robocomp_proxy->getObjectPose(robot_def);

	return {
		(person_pose.position.x-robot_pose.position.x)/1000,
		(person_pose.position.y-robot_pose.position.y)/1000,
		(person_pose.position.z-robot_pose.position.z)/1000
	};
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

void SpecificWorker::update_distance_to_person(std::vector<float> distance)
{
	auto optional_robot_node = G->get_node("robot");
	if (!optional_robot_node.has_value()){
		std::cout << "Robot node not found in DSR." << std::endl;
		return;
	}

	auto optional_person_node = G->get_node("person");
	if (!optional_person_node.has_value()){
		std::cout << "Person node not found in DSR." << std::endl;
		return;
	}

	auto robot_node = optional_robot_node.value();
	auto person_node = optional_person_node.value();

	rt->insert_or_assign_edge_RT(robot_node,person_node.id(),distance, {0.f, 0.f, 0.f});
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
		G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node, (float)0.0);
		G->update_node(robot_node);
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

