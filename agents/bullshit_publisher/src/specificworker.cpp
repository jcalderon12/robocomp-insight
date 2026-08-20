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

	// Delete generated agents if the checkbox is checked
	if (agent_generator_ui.delete_agents_check_box->isChecked()) {
		for (const QString& path : generated_agents) {
			QDir dir(path);
			if (dir.exists()) {
				dir.removeRecursively();
			}
		}
	}
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
	agent_generator_ui.setupUi(&agent_generator_widget);

	// bullshit_publisher UI connections
	connect(bullshit_publisher_ui.create_node_button, &QPushButton::clicked, this, &SpecificWorker::add_node);
	connect(bullshit_publisher_ui.delete_node_button, &QPushButton::clicked, this, &SpecificWorker::delete_node);
	connect(bullshit_publisher_ui.modify_node_button, &QPushButton::clicked, this, &SpecificWorker::modify_node);
	connect(bullshit_publisher_ui.create_edge_button, &QPushButton::clicked, this, &SpecificWorker::add_edge);
	connect(bullshit_publisher_ui.delete_edge_button, &QPushButton::clicked, this, &SpecificWorker::delete_edge);
	connect(bullshit_publisher_ui.modify_edge_button, &QPushButton::clicked, this, &SpecificWorker::modify_edge);	
	connect(bullshit_publisher_ui.create_edge_RT_button, &QPushButton::clicked, this, &SpecificWorker::add_RT_edge);
	connect(bullshit_publisher_ui.delete_edge_RT_button, &QPushButton::clicked, this, &SpecificWorker::delete_RT_edge);
	connect(bullshit_publisher_ui.modify_edge_RT_button, &QPushButton::clicked, this, &SpecificWorker::modify_edge_RT);

	// Selecting from a list box updates its paired text box
	connect(bullshit_publisher_ui.node_list, &QComboBox::currentTextChanged, bullshit_publisher_ui.node_name, &QLineEdit::setText);
	connect(bullshit_publisher_ui.attr_list, &QComboBox::currentTextChanged, bullshit_publisher_ui.attr_name, &QLineEdit::setText);

	// Attr list reflects whichever node is selected as "from"
	connect(bullshit_publisher_ui.node_from_list, &QComboBox::currentTextChanged, this, &SpecificWorker::refresh_attr_list);

	agent_process = new QProcess(this);

	// Connect the create_agent_button to run the agent_generator.py script with the provided agent name
	connect(agent_generator_ui.create_agent_button, &QPushButton::clicked, this, [this](){
		// Clear terminal output
		agent_generator_ui.agent_output->clear();

		// Get the agent name from the text box and validate it
		QString cause_name = agent_generator_ui.agent_name_text->text().trimmed();
		if (cause_name.isEmpty()) {
			agent_generator_ui.agent_status_label->setText("<font color ='red'><b>Error: Nombre vacío</b></font>");
			return;
		}
		
		agent_generator_ui.agent_status_label->setText("<font color ='orange'><b>Generating agent...</b></font>");

		QDir dir(QCoreApplication::applicationDirPath());
		dir.cdUp(); // bin/ -> bullshit_publisher/
		dir.cdUp(); // bullshit_publisher/ -> agents/
		QString output_path = dir.absolutePath();
		QString script_path = output_path + "/agent_generation/agent_generator.py";
		current_agent_name = output_path + "/concept_" + cause_name;
		QStringList args;
		args << script_path << cause_name << output_path;

		agent_generator_ui.agent_output->appendPlainText("Running agent generator script...");

		agent_process->start("python3", args);
	});

	// Connect the agent output to the agent_output text box - standard output
	connect(agent_process, &QProcess::readyReadStandardOutput, this, [this]() {
		QString output = agent_process->readAllStandardOutput().trimmed();
		if (output.isEmpty()) return;

		// Key output in green
		if (output.contains("SUCCESS"))
			agent_generator_ui.agent_output->appendHtml("<font color='green'>SUCCESS: " + output + "</font>");
		// Normal output in black	
		else
			agent_generator_ui.agent_output->appendPlainText(output);
	});	
	
	// Connect the agent error output to the agent_output text box - standard error
	connect(agent_process , &QProcess::readyReadStandardError, this, [this]() {
		QString error = agent_process->readAllStandardError().trimmed();
		if (!error.isEmpty()) 
			agent_generator_ui.agent_output->appendHtml("<font color='red'>ERROR: " + error + "</font>");
	});

	// Connect the finished signal to handle the status label output
	connect(agent_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this, [this](int exit_code, QProcess::ExitStatus exit_status){
		if (exit_code == QProcess::NormalExit && exit_code == 0) {
			agent_generator_ui.agent_status_label->setText("<font color ='green'><b>Agent generated successfully!</b></font>");
			generated_agents.push_back(current_agent_name);
		}
		else {
			agent_generator_ui.agent_status_label->setText("<font color ='red'><b>Error generating agent. Check output for details.</b></font>");
		}
	});

	// Connect the create concept button so it creates a node with text box concept name and a RT edge with its coordinates
	connect(agent_generator_ui.create_concept_button, &QPushButton::clicked, this, [this](){
		QString concept_name = agent_generator_ui.agent_name_text->text().trimmed();
		if (concept_name.isEmpty()) {
			agent_generator_ui.agent_status_label->setText("<font color ='red'><b>Error: Concept name is empty</b></font>");
			return;
		}

		// Create the concept node
		DSR::Node concept_node = DSR::Node::create<object_node_type>(concept_name.toStdString());

		// Use the 'problem' node's computed 3D position if present (see inner_simulator);
		// otherwise fall back to the position captured from the last manual inner_simulator
		// run, so bullshit_publisher/vision_sam/webots can be tested without the full pipeline.
		bool got_position_from_problem_node = false;
		if (auto problem_node = G->get_node("problem"); problem_node.has_value())
		{
			auto it = problem_node.value().attrs().find("problem_position");
			if (it != problem_node.value().attrs().end())
			{
				concept_node.attrs()["problem_position"] = it->second;
				got_position_from_problem_node = true;
			}
		}
		if (!got_position_from_problem_node)
			// X/Y swapped back vs the value captured on the 'problem' node: that one went
			// through get_robot_positions_relative_to_problem's axis swap (inner_simulator/
			// PyBullet convention), while rt_translation (and this projection) use the raw,
			// unswapped Webots/room convention. See conversation for the full trace.
			concept_node.attrs()["problem_position"].value(std::vector<float>{178.89f, -138.89f, 1.00f});

		try {
			G->insert_node(concept_node);
			
			// Create the RT edge from robot to concept node with coordinates
			auto robot_optional = G->get_node("robot");
			if (robot_optional.has_value()) {
				DSR::Node robot_node = robot_optional.value();
				rt->insert_or_assign_edge_RT(robot_node, concept_node.id(), {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});

				refresh_node_combos();
				bullshit_publisher_ui.node_list->setCurrentText(concept_name);
				agent_generator_ui.agent_status_label->setText("<font color ='green'><b>Concept created successfully!</b></font>");
			} else {
				agent_generator_ui.agent_status_label->setText("<font color ='red'><b>Error: Robot node not found</b></font>");
			}
		} catch (const std::exception& e) {
			agent_generator_ui.agent_status_label->setText("<font color ='red'><b>Error creating concept: " + QString::fromStdString(e.what()) + "</b></font>");
		}
	});

	// Add the bullshit_publisher and agent_generator widgets to the graph viewer's dock
	graph_viewer->add_custom_widget_to_dock("Bullshit publisher", &bullshit_publisher_widget);
	graph_viewer->add_custom_widget_to_dock("Agent generator", &agent_generator_widget);

    //initializeCODE

	rt = G->get_rt_api();

	// Edge types this widget can create generically (RT is handled by its own buttons)
	bullshit_publisher_ui.edge_type_list->addItems({"has", "has_intention", "TARGET"});
	refresh_node_combos();

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
	// Create a new node with a default name if the text box is empty, otherwise use the name from the text box
	QString q_node_name = "test_node";
	if (!bullshit_publisher_ui.node_name->text().isEmpty()) {
		q_node_name = bullshit_publisher_ui.node_name->text();
	}

	// Check if the node already exists in the graph
	std::string node_name = q_node_name.toStdString();
	if (G->get_node(node_name).has_value())
		return;

	// Create the node and give it a position for visualization
	DSR::Node test_node = DSR::Node::create<object_node_type>(node_name);
	auto [pos_x, pos_y] = next_layout_position();
	DSR::Attribute pos_x_attr, pos_y_attr;
	pos_x_attr.value(pos_x);
	pos_y_attr.value(pos_y);
	test_node.attrs()["pos_x"] = pos_x_attr;
	test_node.attrs()["pos_y"] = pos_y_attr;

	G->insert_node(test_node);
	refresh_node_combos();
	bullshit_publisher_ui.node_list->setCurrentText(q_node_name);
}


void SpecificWorker::delete_node(){
	// Delete the node with current text box name if it exists in the graph
	QString q_node_name = "test_node";
	if (!bullshit_publisher_ui.node_name->text().isEmpty()) {
		q_node_name = bullshit_publisher_ui.node_name->text();
	}

	// Check if the node exists in the graph
	auto test_optional_node = G->get_node(q_node_name.toStdString());

	// If the node exists, delete it from the graph
	if(test_optional_node.has_value()) {
		G->delete_node(test_optional_node.value().id());
		refresh_node_combos();
	}
}


void SpecificWorker::add_edge(){
	// Create a new edge from "from" to "to", with the selected type, if it does not already exist
	auto test_optional_from = G->get_node(bullshit_publisher_ui.node_from_list->currentText().toStdString());
	auto test_optional_to = G->get_node(bullshit_publisher_ui.node_to_list->currentText().toStdString());
	std::string edge_type = bullshit_publisher_ui.edge_type_list->currentText().toStdString();

	if(test_optional_from.has_value() and test_optional_to.has_value()) {
		DSR::Node node_from = test_optional_from.value();
		DSR::Node node_to = test_optional_to.value();
		auto test_optional_edge = G->get_edge(node_from.id(), node_to.id(), edge_type);
		if(!test_optional_edge.has_value()) {
			DSR::Edge test_edge;
			test_edge.from(node_from.id());
			test_edge.to(node_to.id());
			test_edge.type(edge_type);
			G->add_or_modify_attrib_local<robot_target_x_att>(test_edge, (float)std::experimental::randint(-200, 200));
			G->insert_or_assign_edge(test_edge);
		}
	}
}


void SpecificWorker::delete_edge(){
	// Delete the "from"->"to" edge of the selected type if it exists
	auto test_optional_from = G->get_node(bullshit_publisher_ui.node_from_list->currentText().toStdString());
	auto test_optional_to = G->get_node(bullshit_publisher_ui.node_to_list->currentText().toStdString());
	std::string edge_type = bullshit_publisher_ui.edge_type_list->currentText().toStdString();

	if(test_optional_from.has_value() and test_optional_to.has_value()) {
		DSR::Node node_from = test_optional_from.value();
		DSR::Node node_to = test_optional_to.value();
		auto test_optional_edge = G->get_edge(node_from.id(), node_to.id(), edge_type);
		if(test_optional_edge.has_value()){
			G->delete_edge(node_from.id(), node_to.id(), edge_type);
		}
	}
}

void SpecificWorker::add_RT_edge(){
	// Create a new RT edge from "from" to "to" if it does not already exist
	auto test_optional_from = G->get_node(bullshit_publisher_ui.node_from_list->currentText().toStdString());
	auto test_optional_to = G->get_node(bullshit_publisher_ui.node_to_list->currentText().toStdString());

	if(test_optional_from.has_value() and test_optional_to.has_value())
	{
		DSR::Node node_from = test_optional_from.value();
		DSR::Node node_to = test_optional_to.value();
		auto test_optional_edge = G->get_edge(node_from.id(), node_to.id(), "RT");
		if(!test_optional_edge.has_value())
		{
			rt->insert_or_assign_edge_RT(node_from, node_to.id(), {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});
		}
	}
}


void SpecificWorker::delete_RT_edge(){
	// Delete the "from"->"to" RT edge if it exists
	auto test_optional_from = G->get_node(bullshit_publisher_ui.node_from_list->currentText().toStdString());
	auto test_optional_to = G->get_node(bullshit_publisher_ui.node_to_list->currentText().toStdString());

	if(test_optional_from.has_value() and test_optional_to.has_value())
	{
		DSR::Node node_from = test_optional_from.value();
		DSR::Node node_to = test_optional_to.value();
		auto test_optional_edge = G->get_edge(node_from.id(), node_to.id(), "RT");
		if(test_optional_edge.has_value()){
			G->delete_edge(node_from.id(), node_to.id(), "RT");
		}
	}
}

void SpecificWorker::modify_node(){
	// Modify the position of the specified node if it exists in the graph
	QString q_node_name = "test_node";
	if (!bullshit_publisher_ui.node_name->text().isEmpty()) {
		q_node_name = bullshit_publisher_ui.node_name->text();
	}

	// Check if the node exists in the graph
	std::string node_name = q_node_name.toStdString();
	auto robot_optional_node = G->get_node("robot");
	auto test_optional_node = G->get_node(node_name);

	// If both the node and robot exist, modify the position of the node
	if(robot_optional_node.has_value() && test_optional_node.has_value()) {
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
	// Modify the attribute of the "from"->"to" edge of the selected type if it exists
	auto test_optional_from = G->get_node(bullshit_publisher_ui.node_from_list->currentText().toStdString());
	auto test_optional_to = G->get_node(bullshit_publisher_ui.node_to_list->currentText().toStdString());
	std::string edge_type = bullshit_publisher_ui.edge_type_list->currentText().toStdString();

	if(test_optional_from.has_value() and test_optional_to.has_value()) {
		DSR::Node node_from = test_optional_from.value();
		DSR::Node node_to = test_optional_to.value();
		auto test_optional_edge = G->get_edge(node_from.id(), node_to.id(), edge_type);
		if(test_optional_edge.has_value()) {
			auto test_edge = test_optional_edge.value();
			G->add_or_modify_attrib_local<robot_target_x_att>(test_edge, (float)std::experimental::randint(-200, 200));
			G->insert_or_assign_edge(test_edge);
		}
	}
}

void SpecificWorker::modify_edge_RT(){
	// Modify the "from"->"to" RT edge if it exists
	auto test_optional_from = G->get_node(bullshit_publisher_ui.node_from_list->currentText().toStdString());
	auto test_optional_to = G->get_node(bullshit_publisher_ui.node_to_list->currentText().toStdString());

	if(test_optional_from.has_value() and test_optional_to.has_value()) {
		DSR::Node node_from = test_optional_from.value();
		DSR::Node node_to = test_optional_to.value();
		auto test_optional_edge = G->get_edge(node_from.id(), node_to.id(), "RT");
		if(test_optional_edge.has_value()) {
			rt->insert_or_assign_edge_RT(node_from, node_to.id(),
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

std::pair<float, float> SpecificWorker::next_layout_position()
{
	// Positions new test nodes in rows of 3, relative to the robot's own position
	float robot_pos_x = 0.0f, robot_pos_y = 0.0f;
	try {
		auto robot_opt = G->get_node("robot");
		if (robot_opt.has_value()) {
			auto robot_node = robot_opt.value();
			auto pos_x_it = robot_node.attrs().find("pos_x");
			auto pos_y_it = robot_node.attrs().find("pos_y");
			if (pos_x_it != robot_node.attrs().end())
				if (auto* pf = std::get_if<float>(&pos_x_it->second.value()))
					robot_pos_x = *pf;
			if (pos_y_it != robot_node.attrs().end())
				if (auto* pf = std::get_if<float>(&pos_y_it->second.value()))
					robot_pos_y = *pf;
		}
	} catch (const std::exception& e) {
		std::cerr << "Error getting robot position: " << e.what() << std::endl;
	}

	float pos_x = robot_pos_x + (missions_in_current_row * 300.0f);
	float pos_y = robot_pos_y + current_y_offset;

	missions_in_current_row++;
	if (missions_in_current_row >= 3) {
		missions_in_current_row = 0;
		current_y_offset += 300.0f;
	}

	return {pos_x, pos_y};
}

void SpecificWorker::refresh_node_combos()
{
	bullshit_publisher_ui.node_list->clear();
	bullshit_publisher_ui.node_from_list->clear();
	bullshit_publisher_ui.node_to_list->clear();
	for (const auto& node : G->get_nodes())
	{
		QString name = QString::fromStdString(node.name());
		bullshit_publisher_ui.node_list->addItem(name);
		bullshit_publisher_ui.node_from_list->addItem(name);
		bullshit_publisher_ui.node_to_list->addItem(name);
	}
}

void SpecificWorker::refresh_attr_list()
{
	bullshit_publisher_ui.attr_list->clear();
	auto node_opt = G->get_node(bullshit_publisher_ui.node_from_list->currentText().toStdString());
	if (!node_opt.has_value())
		return;
	for (const auto& attr : node_opt.value().attrs())
		bullshit_publisher_ui.attr_list->addItem(QString::fromStdString(attr.first));
}

