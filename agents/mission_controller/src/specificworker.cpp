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
	connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_node_attrs_slot);
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

	mission_controller_ui.setupUi(&mission_controller_widget);

	graph_viewer->add_custom_widget_to_dock("Mission_controller", &mission_controller_widget);

	model = new MissionModel(this);
	delegate = new MissionDelegate(this);

	mission_controller_ui.mission_list->setModel(model);
	mission_controller_ui.mission_list->setItemDelegate(delegate);

	connect(delegate, &MissionDelegate::missionToggleClicked, this, [this](int row)
	{
		Mission m = model->getMission(row);

		if (m.status == MissionStatus::RUNNING)
		{
			auto now = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - mission_start_time).count();
			mission_accumulated_time += static_cast<int>(elapsed);
			
			model->setMissionStatus(row, MissionStatus::STOPPED);
			on_stopMission_clicked();
		}
		else if (m.status != MissionStatus::COMPLETED)
		{
			active_mission_row = row;
			mission_start_time = std::chrono::steady_clock::now();
			model->setMissionStatus(row, MissionStatus::RUNNING);
			on_startMission_clicked();
		}
	});

	connect(delegate, &MissionDelegate::missionDefaultClicked, this, [this](int row)
	{
		int total_time = mission_accumulated_time;
		
		if (model->getMission(row).status == MissionStatus::RUNNING)
		{
			auto now = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - mission_start_time).count();
			total_time += static_cast<int>(elapsed);
		}
		
		model->setMissionElapsedTime(row, total_time);
		model->setMissionStatus(row, MissionStatus::COMPLETED);
		
		if (active_mission_row == row)
		{
			active_mission_row = -1;
			mission_accumulated_time = 0;
		}
	});

	connect(mission_controller_ui.add_mission_button, &QPushButton::clicked, this, [this]()
	{
		QDialog dialog(&mission_controller_widget);
		Ui::AddMissionDialog dialog_ui;
		dialog_ui.setupUi(&dialog);

		dialog_ui.mission_name->clear();
		for (const auto& mission : getAvailableMissions())
		{
			dialog_ui.mission_name->addItem(QString::fromStdString(mission));
		}

		if (dialog.exec() == QDialog::Accepted)
		{
			QString customName = dialog_ui.mission_custom_name->text();
			QString missionType = dialog_ui.mission_name->currentText();
			
			if (customName.isEmpty())
			{
				customName = missionType;
			}
			
			Mission newMission{customName, missionType};
			model->addMission(newMission);
			on_setMission_clicked();
		}
	});

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

void SpecificWorker::on_setMission_clicked()
{
	// Get robot node
	auto robot_node = G->get_node("robot");
	// Get person node
	auto person_node = G->get_node("person");
	
	if (robot_node.has_value() && person_node.has_value()) {
		DSR::Edge new_target_edge;
		new_target_edge.from(robot_node.value().id());
		new_target_edge.to(person_node.value().id());
		new_target_edge.type("TARGET");
		G->insert_or_assign_edge(new_target_edge);
	}
	else {
		std::cout << "Robot or person node not found in DSR." << std::endl;
	}
}

void SpecificWorker::on_startMission_clicked()
{
	if (std::optional<DSR::Node> optional_node = G->get_node("follow_me"); optional_node.has_value())
	{	
		mission_start_time = std::chrono::steady_clock::now();
		DSR::Node follow_me_node = optional_node.value();
		G->add_or_modify_attrib_local<aff_interacting_att>(follow_me_node, true);
		G->update_node(follow_me_node);
		std::cout << "Affordance activated." << std::endl;
	}
	else{
		std::cout << "Follow me affordance node not found in DSR." << std::endl;
	}
}

void SpecificWorker::on_stopMission_clicked()
{
	if (std::optional<DSR::Node> optional_node = G->get_node("follow_me"); optional_node.has_value())
	{	
		DSR::Node follow_me_node = optional_node.value();
		G->add_or_modify_attrib_local<aff_interacting_att>(follow_me_node, false);
		G->update_node(follow_me_node);
		std::cout << "Affordance stopped." << std::endl;
	}
	else{
		std::cout << "Follow me affordance node not found in DSR." << std::endl;
	}
}

void SpecificWorker::modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names)
{
    auto node = G->get_node(id);
    if (!node.has_value() || node.value().name() != "follow_me")
		return;

    bool relevant = std::any_of(att_names.begin(), att_names.end(), [](const std::string& s){
        return s == "aff_interacting";
    });

    if (!relevant) return;

    auto interacting = G->get_attrib_by_name<aff_interacting_att>(node.value());
    if (!interacting.has_value())
		return;

    bool is_running = interacting.value();

    if (active_mission_row >= 0)
    {
        if (!is_running && model->getMission(active_mission_row).status == MissionStatus::RUNNING)
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - mission_start_time).count();
            mission_accumulated_time += static_cast<int>(elapsed);
        }
        
        model->setMissionStatus(active_mission_row,
            is_running ? MissionStatus::RUNNING : MissionStatus::STOPPED);

        if (!is_running)
            active_mission_row = -1;
    }
}

#pragma endregion