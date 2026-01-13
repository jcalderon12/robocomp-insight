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

    auto rt = G->get_rt_api();

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

}



void SpecificWorker::compute()
{
	switch (agentState)
	{
	case States::IDLE:{
		bool is_target_assigned = check_target_assigned();
		if (is_target_assigned)
			agentState = States::ASSIGNED;
		break;
	}

	case States::ASSIGNED:{
		bool affordance_existed = check_affordance_assigned();
		if (!affordance_existed)
			create_affordance();
		bool affordance_acepted = check_affordance_accepted();
		if (affordance_acepted)
			agentState = States::FOLLOWME;
		break;
	}

	case States::FOLLOWME:{
		auto state = follow_person();
		if (state == "RUNNING")
			break;
		else {
			if (state == "SUCCESS")
				agentState = States::SUCCESS;
			if (state == "FAILED")
				agentState = States::FAILED;
			if (state == "STOPPED")
				agentState = States::STOPPED;
		}
		break;
	}

	case States::SUCCESS:
		std::cout << "Mission success!" << std::endl;
		break;

	case States::FAILED:
		std::cout << "Mission failed!" << std::endl;
		break;

	case States::STOPPED:
		std::cout << "Mission stopped by user!" << std::endl;
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

//SUBSCRIPTION to setVisualObjects method from VisualElementsPub interface
void SpecificWorker::VisualElementsPub_setVisualObjects(RoboCompVisualElementsPub::TData data)
{
//subscribesToCODE

}

/************ DSR Methods *************/

bool SpecificWorker::check_target_assigned()
{
	auto robot_node = G->get_node("robot");
	if (!robot_node.has_value()){
		std::cout << "Robot node not found in DSR." << std::endl;
		return false;
	}
	auto person_nodes = G->get_nodes_by_type("person");
	if (person_nodes.empty()){
		std::cout << "No person nodes found in DSR." << std::endl;
		return false;
	}

	auto target_edges = G->get_edges_by_type("TARGET");
	if (target_edges.empty()){
		std::cout << "No TARGET edge found from robot to person." << std::endl;
		return false;
	}
	else{
		DSR::Edge target_edge = target_edges[0];
		auto target_id = target_edge.to();

		person_node = G->get_node(target_id).value();
		std::cout << "Target assigned to person with id:" << target_id << std::endl;
		return true;
	}
}

bool SpecificWorker::check_affordance_assigned()
{
	auto affordance_nodes = G->get_nodes_by_type("affordance");
	for (const auto& node : affordance_nodes){
		auto edge = G->get_edge(person_node.id(), node.id(), "has_intention");
		if (edge.has_value()){
			std::cout << "Affordance already exists for the assigned target." << std::endl;
			return true;
		}
	}
	std::cout << "No affordance exists for the assigned target." << std::endl;
	return false;
}

void SpecificWorker::create_affordance()
{
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
}

bool SpecificWorker::check_affordance_accepted()
{
	return true;
}

std::string SpecificWorker::follow_person()
{
	return "RUNNING";
}

/**************************************/
// From the RoboCompVisualElementsPub you can use this types:
// RoboCompVisualElementsPub::TObject
// RoboCompVisualElementsPub::TData

