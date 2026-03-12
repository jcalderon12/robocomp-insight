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

	bullshit_publisher_ui.setupUi(&bullshit_publisher_widget);

	connect(bullshit_publisher_ui.create_node_button, &QPushButton::clicked, this, &SpecificWorker::add_node);
	connect(bullshit_publisher_ui.delete_node_button, &QPushButton::clicked, this, &SpecificWorker::delete_node);
	connect(bullshit_publisher_ui.modify_node_button, &QPushButton::clicked, this, &SpecificWorker::modify_node);

	connect(bullshit_publisher_ui.create_edge_button, &QPushButton::clicked, this, &SpecificWorker::add_edge);
	connect(bullshit_publisher_ui.delete_edge_button, &QPushButton::clicked, this, &SpecificWorker::delete_edge);
	connect(bullshit_publisher_ui.modify_edge_button, &QPushButton::clicked, this, &SpecificWorker::modify_edge);
	
	connect(bullshit_publisher_ui.create_edge_RT_button, &QPushButton::clicked, this, &SpecificWorker::add_RT_edge);
	connect(bullshit_publisher_ui.delete_edge_RT_button, &QPushButton::clicked, this, &SpecificWorker::delete_RT_edge);
	connect(bullshit_publisher_ui.modify_edge_RT_button, &QPushButton::clicked, this, &SpecificWorker::modify_edge_RT);

	graph_viewer->add_custom_widget_to_dock("Bullshit publisher", &bullshit_publisher_widget);

    //initializeCODE

	rt = G->get_rt_api();

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

}



void SpecificWorker::compute()
{

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


void SpecificWorker::add_node(){

	auto test_optional_node = G->get_node("test_node");
	if(!test_optional_node.has_value())
	{
		DSR::Node test_node = DSR::Node::create<object_node_type>("test_node");
		G->insert_node(test_node);
	}

}


void SpecificWorker::delete_node(){

	auto test_optional_node = G->get_node("test_node");
	if(test_optional_node.has_value()){
		DSR::Node test_node = test_optional_node.value();
		G->delete_node(test_node.id());
	}
}


void SpecificWorker::add_edge(){

	auto test_optional_node = G->get_node("test_node");
	auto test_optional_robot = G->get_node("robot");
	if(test_optional_node.has_value() and test_optional_robot.has_value())
	{
		DSR::Node test_node = test_optional_node.value();
		DSR::Node test_robot = test_optional_robot.value();
		auto test_optional_edge = G->get_edge(test_robot.id(), test_node.id(), "has");
		if(!test_optional_edge.has_value())
		{
			DSR::Edge test_edge;
			test_edge.from(test_robot.id());
			test_edge.to(test_node.id());
			test_edge.type("has");
			G->add_or_modify_attrib_local<robot_target_x_att>(test_edge, (float)std::experimental::randint(-200, 200));
			G->insert_or_assign_edge(test_edge);
		}

	}

}


void SpecificWorker::delete_edge(){

	auto test_optional_node = G->get_node("test_node");
	auto test_optional_robot = G->get_node("robot");
	if(test_optional_node.has_value() and test_optional_robot.has_value())
	{
		DSR::Node test_node = test_optional_node.value();
		DSR::Node test_robot = test_optional_robot.value();
		auto test_optional_edge = G->get_edge(test_robot.id(), test_node.id(), "has");
		if(test_optional_edge.has_value()){
			G->delete_edge(test_robot.id(), test_node.id(), "has");
		}
	}

}

void SpecificWorker::add_RT_edge(){

	auto test_optional_node = G->get_node("test_node");
	auto test_optional_robot = G->get_node("robot");
	if(test_optional_node.has_value() and test_optional_robot.has_value())
	{
		DSR::Node test_node = test_optional_node.value();
		DSR::Node test_robot = test_optional_robot.value();
		auto test_optional_edge = G->get_edge(test_robot.id(), test_node.id(), "RT");
		if(!test_optional_edge.has_value())
		{
			rt->insert_or_assign_edge_RT(test_robot, test_node.id(), {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});
		}

	}

}


void SpecificWorker::delete_RT_edge(){

	auto test_optional_node = G->get_node("test_node");
	auto test_optional_robot = G->get_node("robot");
	if(test_optional_node.has_value() and test_optional_robot.has_value())
	{
		DSR::Node test_node = test_optional_node.value();
		DSR::Node test_robot = test_optional_robot.value();
		auto test_optional_edge = G->get_edge(test_robot.id(), test_node.id(), "RT");
		if(test_optional_edge.has_value()){
			G->delete_edge(test_robot.id(), test_node.id(), "RT");
		}
	}

}

void SpecificWorker::modify_node(){
	auto robot_optional_node = G->get_node("robot");
	auto test_optional_node = G->get_node("test_node");

	if(robot_optional_node.has_value() && test_optional_node.has_value()){
		DSR::Node robot_node = robot_optional_node.value();
		DSR::Node test_node = test_optional_node.value();
		auto optional_pos_x = G->get_attrib_by_name<pos_x_att>(robot_node.id());
		auto optional_pos_y = G->get_attrib_by_name<pos_y_att>(robot_node.id());
		if(optional_pos_x.has_value() && optional_pos_y.has_value()){
			auto pos_x = optional_pos_x.value();
			auto pos_y = optional_pos_y.value();
			pos_x += (float)std::experimental::randint(-200, 200);
			pos_y += (float)std::experimental::randint(-200, 200);
			G->add_or_modify_attrib_local<pos_x_att>(test_node, pos_x);
			G->add_or_modify_attrib_local<pos_y_att>(test_node, pos_y);
			G->update_node(test_node);
		}
	}
}


void SpecificWorker::modify_edge(){

	auto test_optional_node = G->get_node("test_node");
	auto test_optional_robot = G->get_node("robot");
	if(test_optional_node.has_value() and test_optional_robot.has_value())
	{
		DSR::Node test_node = test_optional_node.value();
		DSR::Node test_robot = test_optional_robot.value();
		auto test_optional_edge = G->get_edge(test_robot.id(), test_node.id(), "has");
		if(test_optional_edge.has_value())
		{
			auto test_edge = test_optional_edge.value();
			G->add_or_modify_attrib_local<robot_target_x_att>(test_edge, (float)std::experimental::randint(-200, 200));
			G->insert_or_assign_edge(test_edge);
		}
	}
}

void SpecificWorker::modify_edge_RT(){
	auto test_optional_node = G->get_node("test_node");
	auto test_optional_robot = G->get_node("robot");
	if(test_optional_node.has_value() and test_optional_robot.has_value())
	{
		DSR::Node test_node = test_optional_node.value();
		DSR::Node test_robot = test_optional_robot.value();
		auto test_optional_edge = G->get_edge(test_robot.id(), test_node.id(), "RT");
		if(test_optional_edge.has_value())
		{
			rt->insert_or_assign_edge_RT(test_robot, test_node.id(), 
				{
				(float)std::experimental::randint(-200, 200), 
				(float)std::experimental::randint(-200, 200), 
				(float)std::experimental::randint(-200, 200)}, 
				{
				(float)std::experimental::randint(-200, 200), 
				(float)std::experimental::randint(-200, 200), 
				(float)std::experimental::randint(-200, 200)}
			);
		}
	}
}
