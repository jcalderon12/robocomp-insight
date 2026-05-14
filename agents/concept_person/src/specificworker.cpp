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

	last_relative_pose = get_person_relative_position();
}



void SpecificWorker::compute()
{
	auto relative_position = get_person_relative_position();


	if (has_significant_change(relative_position, last_relative_pose)){
		if (update_relative_position_to_person(relative_position)){
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

		std::vector<float> p_pos = {person_pose.position.x / 1000.f,
						 person_pose.position.y / 1000.f,
						 person_pose.position.z / 1000.f};
		std::vector<float> r_pos = {robot_pose.position.x / 1000.f,
						 robot_pose.position.y / 1000.f,
						 robot_pose.position.z / 1000.f};

		relative_position = {p_pos[0] - r_pos[0], p_pos[1] - r_pos[1], p_pos[2] - r_pos[2]};
	}
	else{
		auto segmented_objects = this->imagesegmentation_proxy->getSegmentedObjects(true, false);

		std::vector<std::vector<float>> person_positions;

		for (const auto& obj : segmented_objects){
			if (obj.label == "person"){
				auto point_cloud = obj.points3D;
				if (!point_cloud.X.empty()){
					person_positions.push_back(std::vector<float>{point_cloud.X[0], point_cloud.Y[0], point_cloud.Z[0]});
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

bool SpecificWorker::update_relative_position_to_person(const std::vector<float>& relative_position)
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
			if (print_extra_info)
				std::cout << "Large jump confirmation " << consecutive_large_jump_confirmations << "/" << required_large_jump_confirmations << std::endl;
			if (consecutive_large_jump_confirmations < required_large_jump_confirmations)
				return false;
		}
	}

	auto robot_node = optional_robot_node.value();
	auto person_node = optional_person_node.value();
	rt->insert_or_assign_edge_RT(robot_node, person_node.id(), relative_position, {0.0f, 0.0f, 0.0f});
	last_relative_pose = relative_position;
	consecutive_large_jump_confirmations = 0;
	pending_relative_position = last_relative_pose;
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

