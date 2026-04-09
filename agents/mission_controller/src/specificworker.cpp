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
#include <QTimer>
#include <QMessageBox>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <dirent.h>
#include <sys/stat.h>
#include <random>
#include <algorithm>
#include <filesystem>

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
	GenericWorker::initialize();
	G = Graphs.at("work");
	mission_graph = Graphs.at("episodic");
	
	// Load historic graph for debugger
	historic_graph = Graphs.at("debugger");
	
	// Initialize historic manager (viewer is created automatically by config)
	if (historic_graph)
	{
		// Create historic manager with LRU cache size 50
		historic_manager = std::make_unique<HistoricManager>(historic_graph, 50);
		std::cout << "Historic manager initialized" << std::endl;
		
		// Ensure completed_missions folder exists for later use
		system("mkdir -p ./completed_missions");
	}

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
	//graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
	//graph_viewer->add_custom_widget_to_dock("CustomWidget", &custom_widget);

	mission_controller_ui.setupUi(&mission_controller_widget);
	historic_debugger_ui.setupUi(&historic_debugger_widget);

	// Agregar el widget personalizado en el dock de la ventana episodic
	if (!graph_viewers.empty()) {
		graph_viewers.at("episodic")->add_custom_widget_to_dock("Mission Controller", &mission_controller_widget);
		graph_viewers.at("debugger")->add_custom_widget_to_dock("Historic Debugger", &historic_debugger_widget);
		windows.at("debugger")->hide();;
	}

	model = new MissionModel(this);
	delegate = new MissionDelegate(this);

	mission_controller_ui.mission_list->setModel(model);
	mission_controller_ui.mission_list->setItemDelegate(delegate);

	connect(delegate, &MissionDelegate::missionToggleClicked, this, [this](int row)
	{
		Mission m = model->getMission(row);

		if (m.status == MissionStatus::RUNNING)
		{
			// STOP: Mission is currently running, stop it
			auto now = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration<float>(now - mission_start_time).count();
			mission_accumulated_time += elapsed;
			
			model->setMissionStatus(row, MissionStatus::STOPPED);
			model->setMissionElapsedTime(row, mission_accumulated_time);
			
			if (mission_row_to_node_id.count(row) > 0) {
				uint64_t mission_id = mission_row_to_node_id[row];
				mission_scheduler.stopMission(mission_id);
				update_mission_status_episodic(mission_id, "stopped");
				delete_mission_target_edge(mission_id);
				
				try {
					auto mission_opt = mission_graph->get_node(mission_id);
					if (mission_opt.has_value()) {
						auto mission = mission_opt.value();
						DSR::Attribute elapsed_attr;
						elapsed_attr.value(mission_accumulated_time);
						mission.attrs()["elapsed_time"] = elapsed_attr;
						mission_graph->update_node(mission);
						std::cout << "    ✓ Elapsed time saved to mission node: " << mission_accumulated_time << "s" << std::endl;
					}
				} catch (const std::exception& e) {
					std::cerr << "Error saving elapsed time to mission node: " << e.what() << std::endl;
				}
			}
			
			on_stopMission_clicked();
		}
		else if (m.status != MissionStatus::COMPLETED)
		{
			// ACTIVATE: Mission is not running, activate it
			uint64_t new_mission_id = 0;
			if (mission_row_to_node_id.count(row) > 0) {
				new_mission_id = mission_row_to_node_id[row];
			}
			
			if (new_mission_id == 0) {
				std::cerr << "Mission ID not found for row " << row << std::endl;
				return;
			}
			
			// Activate through scheduler (handles preemption automatically)
			auto result = mission_scheduler.activateMission(new_mission_id);
			
			if (!result.success) {
				std::cerr << "Failed to activate mission" << std::endl;
				return;
			}
			
			// If preemption occurred, handle the preempted mission
			if (result.preempted_existing && active_mission_row >= 0) {
				std::cout << "\n[PREEMPTION] Mission \"" << model->getMission(row).name.toStdString() 
					<< "\" is PREEMPTING current mission" << std::endl;
				
				auto now = std::chrono::steady_clock::now();
				auto elapsed = std::chrono::duration<float>(now - mission_start_time).count();
				mission_accumulated_time += elapsed;
				
				model->setMissionStatus(active_mission_row, MissionStatus::STOPPED);
				model->setMissionElapsedTime(active_mission_row, mission_accumulated_time);
				
				update_mission_status_episodic(result.preempted_mission_id, "stopped");
				delete_mission_target_edge(result.preempted_mission_id);
				
				try {
					auto mission_opt = mission_graph->get_node(result.preempted_mission_id);
					if (mission_opt.has_value()) {
						auto mission = mission_opt.value();
						DSR::Attribute elapsed_attr;
						elapsed_attr.value(mission_accumulated_time);
						mission.attrs()["elapsed_time"] = elapsed_attr;
						mission_graph->update_node(mission);
					}
				} catch (const std::exception& e) {
					std::cerr << "Error saving preempted mission time: " << e.what() << std::endl;
				}
			} else if (active_mission_row >= 0 && active_mission_row != row) {
				// No preemption, but another mission was active - stop it gracefully
				auto now = std::chrono::steady_clock::now();
				auto elapsed = std::chrono::duration<float>(now - mission_start_time).count();
				mission_accumulated_time += elapsed;
				
				model->setMissionStatus(active_mission_row, MissionStatus::STOPPED);
				model->setMissionElapsedTime(active_mission_row, mission_accumulated_time);
				
				if (mission_row_to_node_id.count(active_mission_row) > 0) {
					uint64_t prev_mission_id = mission_row_to_node_id[active_mission_row];
					delete_mission_target_edge(prev_mission_id);
					update_mission_status_episodic(prev_mission_id, "stopped");
					
					try {
						auto mission_opt = mission_graph->get_node(prev_mission_id);
						if (mission_opt.has_value()) {
							auto mission = mission_opt.value();
							DSR::Attribute elapsed_attr;
							elapsed_attr.value(mission_accumulated_time);
							mission.attrs()["elapsed_time"] = elapsed_attr;
							mission_graph->update_node(mission);
						}
					} catch (const std::exception& e) {
						std::cerr << "Error saving mission time: " << e.what() << std::endl;
					}
				}
			}
			
			// Activate new mission
			active_mission_row = row;
			mission_start_time = std::chrono::steady_clock::now();
			
			// Read saved elapsed_time to resume
			mission_accumulated_time = 0.0f;
			try {
				auto mission_opt = mission_graph->get_node(new_mission_id);
				if (mission_opt.has_value()) {
					auto mission = mission_opt.value();
					auto elapsed_attr_iter = mission.attrs().find("elapsed_time");
					if (elapsed_attr_iter != mission.attrs().end()) {
						mission_accumulated_time = std::get<float>(elapsed_attr_iter->second.value());
						std::cout << "    ✓ Resumed from saved elapsed time: " << mission_accumulated_time << "s" << std::endl;
					}
				}
			} catch (const std::exception& e) {
				std::cerr << "Error reading mission elapsed_time: " << e.what() << std::endl;
			}
			
			model->setMissionStatus(row, MissionStatus::RUNNING);
			model->setMissionElapsedTime(row, mission_accumulated_time);
			
			// === SYNCHRONIZATION HANDSHAKE ===
			// 1. Create TARGET edge to signal episodic_memory
			create_mission_target_edge(new_mission_id);
			
			// 2. Update mission status to "running" BEFORE waiting for handshake
			//    This allows episodic_memory to detect TARGET + running and enter RECORDING
			update_mission_status_episodic(new_mission_id, "running");
			
			// 3. Wait for episodic_memory to confirm via recording=true
			//    (polling happens in compute() to avoid blocking)
			handshake_waiting_mission_id = new_mission_id;
			handshake_timeout_cycles = 0;
			
			std::cout << "[ACTIVATION] ✓ Mission \"" << model->getMission(row).name.toStdString() 
				<< "\" awaiting episodic_memory handshake..." << std::endl;
			
			// NOTE: on_startMission_clicked() is called by check_recording_handshake() 
			//       in compute() after handshake succeeds (recording=true detected)
			mission_scheduler.setCurrentMission(new_mission_id);
		}
	});

	connect(delegate, &MissionDelegate::missionCompletedClicked, this, [this](int row)
	{
		// Skip if already completed
		if (model->getMission(row).status == MissionStatus::COMPLETED)
		{
			std::cout << "Mission already completed, skipping..." << std::endl;
			return;
		}
		
		float total_time = mission_accumulated_time;
		
		if (model->getMission(row).status == MissionStatus::RUNNING)
		{
			auto now = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration<float>(now - mission_start_time).count();
			total_time += elapsed;
		}
		
		model->setMissionElapsedTime(row, total_time);
		model->setMissionStatus(row, MissionStatus::COMPLETED);
		
		if (active_mission_row == row)
		{
			active_mission_row = -1;
			mission_accumulated_time = 0;
		}
		
		// Update mission status to "completed" in episodic graph
		if (mission_row_to_node_id.count(row) > 0) {
			uint64_t mission_id = mission_row_to_node_id[row];
			update_mission_status_episodic(mission_id, "completed");
			delete_mission_target_edge(mission_id);  // Remove TARGET edge when mission completes
			mission_scheduler.updateMissionStatus(mission_id, "completed");  // Update scheduler
			
			// Optionally add elapsed_time attribute
			try {
				auto mission_opt = mission_graph->get_node(mission_id);
				if (mission_opt.has_value()) {
					auto mission = mission_opt.value();
					DSR::Attribute elapsed_attr;
					elapsed_attr.value(total_time);
					mission.attrs()["elapsed_time"] = elapsed_attr;
					mission_graph->update_node(mission);
				}
			} catch (const std::exception& e) {
				std::cerr << "Could not add elapsed_time attribute: " << e.what() << std::endl;
			}
		}
	});

	connect(mission_controller_ui.add_mission_button, &QPushButton::clicked, this, [this]()
	{
		QDialog dialog(&mission_controller_widget);
		Ui::AddMissionDialog dialog_ui;
		dialog_ui.setupUi(&dialog);

		dialog_ui.mission_name->clear();
		auto available = getAvailableMissions();
		if (available.empty())
		{
			// No predefined missions, allow custom input
			dialog_ui.mission_name->setEditable(true);
		}
		else
		{
			for (const auto& mission : available)
			{
				dialog_ui.mission_name->addItem(QString::fromStdString(mission));
			}
		}

		if (dialog.exec() == QDialog::Accepted)
		{
			QString customName = dialog_ui.mission_custom_name->text();
			QString missionType = dialog_ui.mission_name->currentText();
			int priority = dialog_ui.mission_priority->currentIndex();  // 0=Critical, 1=High, 2=Normal, 3=Low, 4=Very Low
			
			if (customName.isEmpty())
			{
				customName = missionType;
			}
			
			// Convert priority index to priority value: Critical=5, High=4, Normal=3, Low=2, VeryLow=1
			int priority_value = 5 - priority;
			
			Mission newMission{customName, missionType, 0.0f, MissionStatus::IDLE, priority_value};
			int row = model->rowCount();  // Get future row index
			model->addMission(newMission);
			
			// Insert mission node in episodic graph with status="pending"
			insert_mission_node_episodic(customName.toStdString(), row, priority_value);
			
			on_setMission_clicked();
		}
	});
	// Configurar timer para actualizar tiempo en tiempo real
	mission_timer = new QTimer(this);

	// Connect debugger scrollbars and search
	connect(historic_debugger_ui.local_changes_scroll_bar, &QScrollBar::valueChanged, this, &SpecificWorker::local_changes_management);
	connect(historic_debugger_ui.global_changes_scroll_bar, &QScrollBar::valueChanged, this, &SpecificWorker::global_changes_management);
	connect(historic_debugger_ui.time_input, &QLineEdit::returnPressed, this, &SpecificWorker::on_time_search);
	
	// Connect load_in_debugger button if it exists
	if (mission_controller_ui.load_in_debugger_button)
	{
		// Always enabled - check mission status when clicked
		mission_controller_ui.load_in_debugger_button->setEnabled(true);
		
		connect(mission_controller_ui.load_in_debugger_button, &QPushButton::clicked, this, [this]()
		{
			QModelIndex current_index = mission_controller_ui.mission_list->currentIndex();
			if (current_index.isValid())
			{
				load_mission_in_debugger(current_index.row());
			}
			else
			{
				QMessageBox::warning(&mission_controller_widget, "Warning", "Please select a mission first");
			}
		});
	}
	
	// Connect autopilot toggle button
	connect(mission_controller_ui.autopilot_toggle_button, &QPushButton::clicked, this, [this]()
	{
		autopilot_enabled = !autopilot_enabled;
		
		// Update button appearance and text
		if (autopilot_enabled) {
			// Initialize state machine
			autopilot_state = AutopilotState::SELECTING_NEXT;
			idle_cycles_count = 0;
			current_active_mission_id = 0;
			current_active_mission_type = "";
			
			mission_controller_ui.autopilot_toggle_button->setText("ON");
			mission_controller_ui.autopilot_toggle_button->setStyleSheet(
				"QPushButton {"
				"    background-color: #4CAF50;"
				"    color: white;"
				"    border: none;"
				"    border-radius: 5px;"
				"    padding: 8px 16px;"
				"    font-weight: bold;"
				"}"
				"QPushButton:hover {"
				"    background-color: #45a049;"
				"}"
				"QPushButton:pressed {"
				"    background-color: #3d8b40;"
				"}"
			);
			std::cout << "[AUTOPILOT] ✓ Autopilot mode ENABLED - State machine initialized" << std::endl;
		} else {
			mission_controller_ui.autopilot_toggle_button->setText("OFF");
			mission_controller_ui.autopilot_toggle_button->setStyleSheet(
				"QPushButton {"
				"    background-color: #f44336;"
				"    color: white;"
				"    border: none;"
				"    border-radius: 5px;"
				"    padding: 8px 16px;"
				"    font-weight: bold;"
				"}"
				"QPushButton:hover {"
				"    background-color: #da190b;"
				"}"
				"QPushButton:pressed {"
				"    background-color: #c91c00;"
				"}"
			);
			std::cout << "[AUTOPILOT] ✗ Autopilot mode DISABLED" << std::endl;
		}
	});
	
	std::cout << "Historic debugger widget initialized" << std::endl;

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

	// Initialize mission timer for real-time updates
	mission_timer = new QTimer(this);
	connect(mission_timer, &QTimer::timeout, this, &SpecificWorker::updateMissionTime);
	mission_timer->start(10); 
}



void SpecificWorker::compute()
{
	// === SYNCHRONIZATION HANDSHAKE ===
	// Check if episodic_memory confirmed recording state (polling every compute cycle)
	if (handshake_waiting_mission_id != 0) {
		check_recording_handshake();
	}
	
	// === AUTOPILOT LOGIC ===
    // Autopilot state machine orchestration
	if (autopilot_enabled)
	{
		autopilot_step();
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
	}
	else{
	}
}

void SpecificWorker::on_stopMission_clicked()
{
	if (std::optional<DSR::Node> optional_node = G->get_node("follow_me"); optional_node.has_value())
	{	
		DSR::Node follow_me_node = optional_node.value();
		G->add_or_modify_attrib_local<aff_interacting_att>(follow_me_node, false);
		G->update_node(follow_me_node);
	}
	else{
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
            auto elapsed = std::chrono::duration<float>(now - mission_start_time).count();
            mission_accumulated_time += elapsed;
        }
        
        model->setMissionStatus(active_mission_row,
            is_running ? MissionStatus::RUNNING : MissionStatus::STOPPED);

        if (!is_running)
            active_mission_row = -1;
    }
}

void SpecificWorker::updateMissionTime()
{
    if (active_mission_row >= 0)
    {
        Mission m = model->getMission(active_mission_row);
        
        if (m.status == MissionStatus::RUNNING)
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<float>(now - mission_start_time).count();
            float totalTime = mission_accumulated_time + elapsed;
            
            model->setMissionElapsedTime(active_mission_row, totalTime);
        }
    }
}

// ============= DEBUGGER METHODS =============

void SpecificWorker::load_mission_changes(const std::string filename) {
	std::ifstream in_file(filename);
	if (!in_file.is_open()) { 
		std::cerr << "Error: File " << filename << " couldn't be open for reading." << std::endl; 
		return; 
	}

	std::string line;
	int line_count = 0;
	while (std::getline(in_file, line)) {
		line_count++;

		if (line.empty())
			continue;

		try {
			if (auto data_ptr = DSRDecoder::decode(line); data_ptr)
				decoded_data[data_ptr->timestamp] = *data_ptr;
		} 
		catch (const std::exception &e) {
			std::cerr << "Error decoding line " << line_count << ": " << e.what() << std::endl;
		}
	}

	in_file.close();
	std::cout << "File " << filename << " loaded with " << decoded_data.size() << " events." << std::endl;
	
	if (historic_manager && !decoded_data.empty())
	{
		historic_manager->index_file(filename);
		std::cout << "Historic manager indexed " << historic_manager->get_keyframe_count() << " keyframes" << std::endl;
		
		// Configure scrollbars to match loaded mission
		size_t keyframe_count = historic_manager->get_keyframe_count();
		if (keyframe_count > 0)
		{
			historic_debugger_ui.global_changes_scroll_bar->blockSignals(true);
			historic_debugger_ui.global_changes_scroll_bar->setMaximum(static_cast<int>(keyframe_count - 1));
			historic_debugger_ui.global_changes_scroll_bar->setValue(0);
			historic_debugger_ui.global_changes_scroll_bar->blockSignals(false);
			
			// Load first keyframe
			global_changes_management(0);
		}
	}
}

void SpecificWorker::display_debugger_graph() {
	if (!historic_manager || historic_value < 0) return;
	
	size_t keyframe_idx = static_cast<size_t>(historic_value);
	if (keyframe_idx >= historic_manager->get_keyframe_count()) return;
	
	historic_manager->load_keyframe(keyframe_idx, true);
	std::cout << "Displaying keyframe " << keyframe_idx << std::endl;
}

void SpecificWorker::local_changes_management(int value) {
	if (!historic_manager) return;
	
	int keyframe_idx = historic_debugger_ui.global_changes_scroll_bar->value();
	size_t local_count = historic_manager->get_local_changes_count(static_cast<size_t>(keyframe_idx));

	historic_manager->load_keyframe(static_cast<size_t>(keyframe_idx), false);

	if (value > 0)
		historic_manager->apply_local_changes_up_to(static_cast<size_t>(keyframe_idx), static_cast<size_t>(value - 1));

	if (value > 0 && local_count > 0) {
		auto [ts, type] = historic_manager->get_local_change_info(static_cast<size_t>(keyframe_idx), static_cast<size_t>(value - 1));
		historic_debugger_ui.local_display_label->setText(
			QString("%1 / %2 - %3").arg(value).arg(local_count).arg(QString::fromStdString(type)));
		uint64_t t0 = historic_manager->get_keyframe_timestamp(0);
		double seconds = (ts > t0) ? static_cast<double>(ts - t0) / 1e9 : 0.0;
		historic_debugger_ui.time_input->setText(QString::number(seconds, 'f', 3));
	} else {
		historic_debugger_ui.local_display_label->setText(QString("0 / %1").arg(local_count));
		uint64_t t0 = historic_manager->get_keyframe_timestamp(0);
		uint64_t ts = historic_manager->get_keyframe_timestamp(static_cast<size_t>(keyframe_idx));
		double seconds = (ts > t0) ? static_cast<double>(ts - t0) / 1e9 : 0.0;
		historic_debugger_ui.time_input->setText(QString::number(seconds, 'f', 3));
	}
}

void SpecificWorker::global_changes_management(int value) {
	if (!historic_manager) return;
	
	size_t total = historic_manager->get_keyframe_count();
	if (value < 0 || static_cast<size_t>(value) >= total) return;  // Validate range
	
	historic_manager->load_keyframe(static_cast<size_t>(value), false);
	update_local_scrollbar(static_cast<size_t>(value));

	historic_debugger_ui.local_changes_scroll_bar->blockSignals(true);
	historic_debugger_ui.local_changes_scroll_bar->setValue(0);
	historic_debugger_ui.local_changes_scroll_bar->blockSignals(false);

	historic_debugger_ui.global_display_label->setText(QString("%1 / %2").arg(value + 1).arg(total));

	uint64_t t0 = historic_manager->get_keyframe_timestamp(0);
	uint64_t ts = historic_manager->get_keyframe_timestamp(static_cast<size_t>(value));
	double seconds = (ts > t0) ? static_cast<double>(ts - t0) / 1e9 : 0.0;
	historic_debugger_ui.time_input->setText(QString::number(seconds, 'f', 3));
}

void SpecificWorker::update_local_scrollbar(size_t keyframe_idx) {
	if (!historic_manager) return;
	
	size_t local_count = historic_manager->get_local_changes_count(keyframe_idx);
	size_t total_kf = historic_manager->get_keyframe_count();

	historic_debugger_ui.local_changes_scroll_bar->setMaximum(static_cast<int>(local_count));
	historic_debugger_ui.local_display_label->setText(QString("0 / %1").arg(local_count));
	historic_debugger_ui.global_display_label->setText(QString("%1 / %2").arg(keyframe_idx + 1).arg(total_kf));
}

void SpecificWorker::on_time_search() {
	if (!historic_manager) return;
	
	QString time_str = historic_debugger_ui.time_input->text();
	bool ok = false;
	double seconds = time_str.toDouble(&ok);

	if (!ok) {
		std::cerr << "Invalid time input: " << time_str.toStdString() << std::endl;
		return;
	}

	uint64_t t0 = historic_manager->get_keyframe_timestamp(0);
	uint64_t target_timestamp = t0 + static_cast<uint64_t>(seconds * 1e9);

	std::cout << "Searching for time " << seconds << "s (ns: " << target_timestamp << ")" << std::endl;

	auto keyframe_idx_opt = historic_manager->find_keyframe_at_or_before(target_timestamp);
	if (!keyframe_idx_opt.has_value()) {
		std::cerr << "No keyframe found before target timestamp" << std::endl;
		return;
	}

	size_t keyframe_idx = keyframe_idx_opt.value();
	uint64_t kf_timestamp = historic_manager->get_keyframe_timestamp(keyframe_idx);

	historic_manager->load_keyframe(keyframe_idx, false);

	historic_debugger_ui.global_changes_scroll_bar->blockSignals(true);
	historic_debugger_ui.global_changes_scroll_bar->setValue(static_cast<int>(keyframe_idx));
	historic_debugger_ui.global_changes_scroll_bar->blockSignals(false);

	size_t total_kf = historic_manager->get_keyframe_count();
	historic_debugger_ui.global_display_label->setText(QString("%1 / %2").arg(keyframe_idx + 1).arg(total_kf));

	size_t local_count = historic_manager->get_local_changes_count(keyframe_idx);
	historic_debugger_ui.local_changes_scroll_bar->setMaximum(static_cast<int>(local_count));

	auto local_change_idx_opt = historic_manager->find_local_change_closest(keyframe_idx, target_timestamp);
	uint64_t actual_timestamp = kf_timestamp;

	if (local_change_idx_opt.has_value()) {
		size_t local_change_idx = local_change_idx_opt.value();
		auto [ts, type] = historic_manager->get_local_change_info(keyframe_idx, local_change_idx);

		historic_manager->apply_local_changes_up_to(keyframe_idx, local_change_idx);

		historic_debugger_ui.local_changes_scroll_bar->blockSignals(true);
		historic_debugger_ui.local_changes_scroll_bar->setValue(static_cast<int>(local_change_idx + 1));
		historic_debugger_ui.local_changes_scroll_bar->blockSignals(false);

		historic_debugger_ui.local_display_label->setText(QString("%1 / %2 - %3").arg(local_change_idx + 1).arg(local_count).arg(QString::fromStdString(type)));
		actual_timestamp = ts;
	} else {
		historic_debugger_ui.local_changes_scroll_bar->blockSignals(true);
		historic_debugger_ui.local_changes_scroll_bar->setValue(0);
		historic_debugger_ui.local_changes_scroll_bar->blockSignals(false);
		historic_debugger_ui.local_display_label->setText(QString("0 / %1").arg(local_count));
	}

	double actual_seconds = static_cast<double>(actual_timestamp - t0) / 1e9;
	historic_debugger_ui.time_input->setText(QString::number(actual_seconds, 'f', 3));

	std::cout << "Time search completed" << std::endl;
}

void SpecificWorker::load_mission_in_debugger(int row) {
	if (!model) return;
	
	Mission m = model->getMission(row);
	std::string mission_name_std = m.name.toStdString();
	
	// Check if mission is completed before loading
	bool mission_completed = false;
	if (mission_graph && mission_row_to_node_id.count(row) > 0) {
		try {
			uint64_t mission_id = mission_row_to_node_id[row];
			auto mission_opt = mission_graph->get_node(mission_id);
			if (mission_opt.has_value()) {
				auto mission = mission_opt.value();
				auto status_attr = mission.attrs().find("status");
				if (status_attr != mission.attrs().end()) {
					try {
						std::string status = std::get<std::string>(status_attr->second.value());
						mission_completed = (status == "completed");
						if (!mission_completed) {
							QMessageBox::warning(&mission_controller_widget, "Warning",
								"Mission '" + QString::fromStdString(mission_name_std) + "' is not completed.\n\n" +
								"Only completed missions can be loaded in the debugger.");
							return;
						}
					} catch (const std::bad_variant_access& e) {
						std::cerr << "Error getting status value: " << e.what() << std::endl;
						mission_completed = false;
					}
				}
			}
		} catch (const std::exception& e) {
			std::cerr << "Error checking mission status: " << e.what() << std::endl;
			mission_completed = false;
		}
	} else {
		// If we can't check status, allow loading (mission_graph not ready or mission_id not mapped)
		mission_completed = true;
	}
	
	std::string most_recent_mission = "";
	
	// IMPORTANT: Give episodic_memory time to write the file (200ms delay)
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	
	std::string mission_name_safe = mission_name_std;
	std::replace(mission_name_safe.begin(), mission_name_safe.end(), ' ', '_');
	
	// Search paths (relative first, as they work from any location)
	std::vector<std::string> search_paths = {
		"../episodic_memory/recorded_missions",
		"./recorded_missions",
		"/home/robolab/robocomp/components/robocomp-insight/agents/episodic_memory/recorded_missions"
	};
	
	time_t most_recent_time = 0;
	std::cout << "Loading mission '" << mission_name_std << "' in debugger..." << std::endl;
	
	for (const auto& search_path : search_paths)
	{
		DIR* dir = opendir(search_path.c_str());
		if (dir != nullptr)
		{
			struct dirent* entry;
			
			while ((entry = readdir(dir)) != nullptr)
			{
				std::string filename = entry->d_name;
				
				// Match mission_name_ pattern
				if (filename.find("mission_" + mission_name_safe + "_") == 0 && filename.find(".txt") != std::string::npos)
				{
					std::string filepath = search_path + "/" + filename;
					struct stat file_stat;
					
					if (stat(filepath.c_str(), &file_stat) == 0)
					{
						if (file_stat.st_mtime > most_recent_time)
						{
							most_recent_time = file_stat.st_mtime;
							most_recent_mission = filepath;
						}
					}
				}
			}
			
			closedir(dir);
		}
	}
	
	if (!most_recent_mission.empty())
	{
		// Verify the file exists and can be read
		struct stat buffer;
		if (stat(most_recent_mission.c_str(), &buffer) == 0) {
			decoded_data.clear();
			load_mission_changes(most_recent_mission);
			std::cout << "✓ Mission loaded in debugger with " << historic_manager->get_keyframe_count() << " keyframes" << std::endl;
			
			// Show debugger window and bring to front
			if (windows.find("debugger") != windows.end())
			{
				windows.at("debugger").get()->show();
				windows.at("debugger").get()->raise();
				windows.at("debugger").get()->activateWindow();
			}
		} else {
			QMessageBox::warning(&mission_controller_widget, "Error", 
				"Mission file found but cannot be read:\n" +
				QString::fromStdString(most_recent_mission) + "\n\n" +
				"Check file permissions.");
		}
	}
	else
	{
		QMessageBox::warning(&mission_controller_widget, "Error", 
			"No mission file found for '" + QString::fromStdString(mission_name_std) + "'\n\n" +
			"Make sure episodic_memory has saved the mission.");
	}
}

// ===== METHODS FOR MISSION MANAGEMENT IN EPISODIC GRAPH =====

std::optional<uint64_t> SpecificWorker::insert_mission_node_episodic(const std::string &mission_name, int row, int priority) {
	try {
		// Generate unique mission name with timestamp and random number
		auto now = std::chrono::system_clock::now();
		auto time_t_now = std::chrono::system_clock::to_time_t(now);
		std::stringstream ss;
		ss << std::put_time(std::localtime(&time_t_now), "%d%m%Y_%H%M%S");
		std::string timestamp_str = ss.str();
		
		// Add random number for uniqueness
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<> dis(1000, 9999);
		int random_num = dis(gen);
		
		std::string unique_mission_name = mission_name + "-" + timestamp_str + "-" + std::to_string(random_num);
		
		// Replace spaces with underscores in mission name for file naming
		std::string mission_name_safe = mission_name;
		std::replace(mission_name_safe.begin(), mission_name_safe.end(), ' ', '_');
		
		// Generate filepath (absolute path)
		std::filesystem::path missions_dir = std::filesystem::current_path() / "recorded_missions";
		std::filesystem::create_directories(missions_dir);
		
		std::string filename = "mission_" + mission_name_safe + "_" + timestamp_str + ".txt";
		std::filesystem::path filepath = missions_dir / filename;
		std::string filepath_str = filepath.string();
		
		// Pre-create mission file (empty)
		std::ofstream mission_file(filepath_str, std::ios::app);
		if (!mission_file.is_open()) {
			std::cerr << "Failed to create mission file: " << filepath_str << std::endl;
			return std::nullopt;
		}
		mission_file.close();
		std::cout << "Pre-created mission file: " << filepath_str << std::endl;
		
		// Create mission node with unique name
		DSR::Node mission_node;
		mission_node.type("intention");
		mission_node.name(unique_mission_name);
		mission_node.agent_id(agent_id);
		
		// Add status attribute (initially "pending")
		DSR::Attribute status_attr;
		status_attr.value("pending");
		mission_node.attrs()["status"] = status_attr;
		
		// Add filepath attribute (absolute path where data will be saved)
		DSR::Attribute filepath_attr;
		filepath_attr.value(filepath_str);
		mission_node.attrs()["filepath"] = filepath_attr;
		
		// Add priority attribute (from dialog: 1-5)
		DSR::Attribute priority_attr;
		priority_attr.value(priority);
		mission_node.attrs()["priority"] = priority_attr;
		
		// Add elapsed_time attribute (initially 0.0)
		DSR::Attribute elapsed_time_attr;
		elapsed_time_attr.value(0.0f);
		mission_node.attrs()["elapsed_time"] = elapsed_time_attr;
		
		// === Calculate position for visualization ===
		// Get robot position (default to 0,0 if not found)
		float robot_pos_x = 0.0f;
		float robot_pos_y = 0.0f;
		try {
			auto robot_opt = mission_graph->get_node("robot");
			if (robot_opt.has_value()) {
				auto robot = robot_opt.value();
				// Try to get pos_x and pos_y attributes
				auto pos_x_it = robot.attrs().find("pos_x");
				auto pos_y_it = robot.attrs().find("pos_y");
				if (pos_x_it != robot.attrs().end()) {
					if (auto* pf = std::get_if<float>(&pos_x_it->second.value())) {
						robot_pos_x = *pf;
					}
				}
				if (pos_y_it != robot.attrs().end()) {
					if (auto* pf = std::get_if<float>(&pos_y_it->second.value())) {
						robot_pos_y = *pf;
					}
				}
			}
		} catch (const std::exception& e) {
			std::cerr << "Error getting robot position: " << e.what() << std::endl;
		}
		
		// Calculate mission position: arrange in rows of 5 missions
		float mission_pos_x = robot_pos_x + (missions_in_current_row * 150.0f);  // 150 units between missions
		float mission_pos_y = robot_pos_y + current_y_offset;  // Below robot (positive Y direction)
		
		// Add position attributes
		DSR::Attribute pos_x_attr;
		pos_x_attr.value(mission_pos_x);
		mission_node.attrs()["pos_x"] = pos_x_attr;
		
		DSR::Attribute pos_y_attr;
		pos_y_attr.value(mission_pos_y);
		mission_node.attrs()["pos_y"] = pos_y_attr;
		
		// Update layout counters
		missions_in_current_row++;
		if (missions_in_current_row >= 5) {
			missions_in_current_row = 0;
			current_y_offset += 100.0f;  // Move 100 units down for next row
		}
		
		auto mission_id_opt = mission_graph->insert_node(mission_node);
		
		if (mission_id_opt.has_value()) {
			uint64_t mission_id = mission_id_opt.value();
			mission_row_to_node_id[row] = mission_id;
			std::cout << "Mission node created in episodic graph: " << unique_mission_name << " (id: " << mission_id << ")" << std::endl;
			std::cout << "  ✓ Unique name: " << unique_mission_name << std::endl;
			std::cout << "  ✓ Filepath: " << filepath_str << std::endl;
			std::cout << "  ✓ Priority: " << priority << std::endl;
		std::cout << "  ✓ Position: (" << mission_pos_x << ", " << mission_pos_y << ")" << std::endl;
		
		// Add has_intention edge from robot to mission
		try {
			auto robot_opt = mission_graph->get_node("robot");
			if (robot_opt.has_value()) {
				DSR::Edge has_intention_edge;
				has_intention_edge.from(robot_opt.value().id());
				has_intention_edge.to(mission_id);
				has_intention_edge.type("has_intention");
				mission_graph->insert_or_assign_edge(has_intention_edge);
			}
		} catch (const std::exception& e) {
			std::cerr << "Exception creating has_intention edge: " << e.what() << std::endl;
		}
			
			// Add mission to scheduler (all new missions are USER-initiated by default)
			mission_scheduler.addMission(mission_id, priority, MissionType::USER);
			std::cout << "  ✓ Mission added to scheduler with priority " << priority << std::endl;
			
			return mission_id;
		} else {
			std::cerr << "Failed to insert mission node: " << unique_mission_name << std::endl;
			return std::nullopt;
		}
	} catch (const std::exception& e) {
		std::cerr << "Exception inserting mission node: " << e.what() << std::endl;
		return std::nullopt;
	}
}

void SpecificWorker::update_mission_status_episodic(uint64_t mission_id, const std::string &status) {
	try {
		auto mission_opt = mission_graph->get_node(mission_id);
		if (!mission_opt.has_value()) {
			std::cerr << "[STATUS_UPDATE_ERROR] Mission node not found: " << mission_id << std::endl;
			return;
		}
		
		auto mission = mission_opt.value();
		
		// Verify CURRENT status BEFORE change
		auto current_status_iter = mission.attrs().find("status");
		std::string current_status = (current_status_iter != mission.attrs().end()) ? 
		                              std::get<std::string>(current_status_iter->second.value()) : "unknown";
		std::cout << "[STATUS_UPDATE_START] Mission id: " << mission_id << ", Current status: '" << current_status 
		          << "', New status: '" << status << "'" << std::endl;
		
		DSR::Attribute status_attr;
		status_attr.value(status);
		mission.attrs()["status"] = status_attr;
		
		mission_graph->update_node(mission);
		std::cout << "[STATUS_UPDATE_EXECUTE] update_node() called" << std::endl;
		
		// VERIFY the update actually happened
		auto mission_verify = mission_graph->get_node(mission_id);
		if (mission_verify.has_value()) {
			auto verified_mission = mission_verify.value();
			auto verified_status_iter = verified_mission.attrs().find("status");
			if (verified_status_iter != verified_mission.attrs().end()) {
				std::string verified_status = std::get<std::string>(verified_status_iter->second.value());
				if (verified_status == status) {
					std::cout << "[STATUS_UPDATE_SUCCESS] ??? Status verified in graph: '" << verified_status << "'" << std::endl;
				} else {
					std::cout << "[STATUS_UPDATE_MISMATCH] ??? Verified status '" << verified_status << "' != requested '" << status << "'" << std::endl;
				}
			}
		}
	} catch (const std::exception& e) {
		std::cerr << "[STATUS_UPDATE_EXCEPTION] Exception updating mission status: " << e.what() << std::endl;
	}
}

void SpecificWorker::create_mission_target_edge(uint64_t mission_id) {
	try {
		auto robot_opt = mission_graph->get_node("robot");
		if (!robot_opt.has_value()) {
			std::cerr << "Robot node not found in episodic graph" << std::endl;
			return;
		}
		
		
		DSR::Edge target_edge;
		target_edge.from(robot_opt.value().id());
		target_edge.to(mission_id);
		target_edge.type("TARGET");
		
		mission_graph->insert_or_assign_edge(target_edge);
		
		// Verify the edge was actually created
		auto all_target_edges = mission_graph->get_edges_by_type("TARGET");
		int target_count = 0;
		for (const auto& edge : all_target_edges) {
			if (edge.from() == robot_opt.value().id() && edge.to() == mission_id) {
				target_count++;
			}
		}
		
	} catch (const std::exception& e) {
	}
}

void SpecificWorker::delete_mission_target_edge(uint64_t mission_id) {
	try {
		auto robot_opt = mission_graph->get_node("robot");
		if (!robot_opt.has_value()) {
			std::cerr << "Robot node not found in episodic graph" << std::endl;
			return;
		}
		
		
		// Count TARGET edges BEFORE deletion
		auto all_target_edges_before = mission_graph->get_edges_by_type("TARGET");
		int target_count_before = 0;
		for (const auto& edge : all_target_edges_before) {
			if (edge.from() == robot_opt.value().id() && edge.to() == mission_id) {
				target_count_before++;
			}
		}
		
		mission_graph->delete_edge(robot_opt.value().id(), mission_id, "TARGET");
		
		// Count TARGET edges AFTER deletion
		auto all_target_edges_after = mission_graph->get_edges_by_type("TARGET");
		int target_count_after = 0;
		for (const auto& edge : all_target_edges_after) {
			if (edge.from() == robot_opt.value().id() && edge.to() == mission_id) {
				target_count_after++;
			}
		}
		
		if (target_count_after == 0) {
		} else {
		}
		
	} catch (const std::exception& e) {
	}
}

// ===== AUTOPILOT HELPER METHODS - FACTORIZED =====

void SpecificWorker::create_or_check_follow_person_mission()
{
	// Only proceed if we're not already waiting for affordance
	if (waiting_mission_row >= 0)
		return;
	
	// Check if follow_person mission already exists
	for (int i = 0; i < model->rowCount(); i++)
	{
		if (model->getMission(i).type == "follow_person")
		{
			std::cout << "[AUTOPILOT] follow_person mission already exists at row " << i << std::endl;
			return;
		}
	}
	
	// Create follow_person mission
	std::cout << "[AUTOPILOT] Creating follow_person mission..." << std::endl;
	
	QString customName = "follow_person_autopilot";
	QString missionType = "follow_person";
	int priority_value = 3;  // Normal priority
	
	Mission newMission{customName, missionType, 0.0f, MissionStatus::IDLE, priority_value};
	int row = model->rowCount();
	model->addMission(newMission);
	
	// Insert mission node in episodic graph
	auto mission_id_opt = insert_mission_node_episodic(customName.toStdString(), row, priority_value);
	
	if (mission_id_opt.has_value())
	{
		waiting_mission_row = row;
		waiting_mission_id = mission_id_opt.value();
		
		// Create TARGET edge (same as on_setMission_clicked)
		auto robot_opt = G->get_node("robot");
		auto person_opt = G->get_node("person");
		
		if (robot_opt.has_value() && person_opt.has_value())
		{
			DSR::Edge new_target_edge;
			new_target_edge.from(robot_opt.value().id());
			new_target_edge.to(person_opt.value().id());
			new_target_edge.type("TARGET");
			G->insert_or_assign_edge(new_target_edge);
		}
		
		std::cout << "[AUTOPILOT] ✓ Mission created (id: " << waiting_mission_id << ")" << std::endl;
		std::cout << "[AUTOPILOT] Waiting for affordance 'follow_me'..." << std::endl;
	}
	else
	{
		std::cerr << "[AUTOPILOT] ✗ Failed to create mission" << std::endl;
	}
}

void SpecificWorker::check_affordance_and_activate()
{
	// Only proceed if we're waiting for affordance
	if (waiting_mission_row < 0 || waiting_mission_id == 0)
		return;
	
	auto affordance_opt = G->get_node("follow_me");
	if (!affordance_opt.has_value())
		return;
	
	auto affordance = affordance_opt.value();
	auto aff_iter = affordance.attrs().find("aff_interacting");
	if (aff_iter == affordance.attrs().end())
		return;
	
	// Affordance exists with aff_interacting - activate mission
	std::cout << "[AUTOPILOT] ✓ Affordance 'follow_me' detected!" << std::endl;
	
	int row = waiting_mission_row;
	uint64_t mission_id = waiting_mission_id;
	
	// Activate mission through scheduler (handles preemption automatically)
	auto result = mission_scheduler.activateMission(mission_id);
	
	if (!result.success) {
		std::cerr << "[AUTOPILOT] ✗ Failed to activate mission" << std::endl;
		return;
	}
	
	active_mission_row = row;
	mission_start_time = std::chrono::steady_clock::now();
	mission_accumulated_time = 0.0f;
	
	// Update model and graph - UI changes
	model->setMissionStatus(row, MissionStatus::RUNNING);
	update_mission_status_episodic(mission_id, "running");
	
	QModelIndex idx = model->index(row, 0);
	mission_controller_ui.mission_list->update(idx);
	
	// Set aff_interacting to true
	on_startMission_clicked();
	
	follow_person_active = true;
	last_aff_interacting_state = true;
	
	std::cout << "[AUTOPILOT] ✓✓ Mission activated!" << std::endl;
	
	// Clear waiting state
	waiting_mission_row = -1;
	waiting_mission_id = 0;
}

void SpecificWorker::monitor_aff_interacting_state()
{
	// Only monitor if follow_person is active
	if (!follow_person_active || active_mission_row < 0)
		return;
	
	auto affordance_opt = G->get_node("follow_me");
	if (!affordance_opt.has_value())
		return;
	
	auto affordance = affordance_opt.value();
	auto aff_iter = affordance.attrs().find("aff_interacting");
	if (aff_iter == affordance.attrs().end())
		return;
	
	try {
		bool current_state = std::get<bool>(aff_iter->second.value());
		
		// Detect change from true to false
		if (last_aff_interacting_state && !current_state)
		{
			on_aff_interacting_false();
		}
		
		// Update tracking state
		last_aff_interacting_state = current_state;
		
	} catch (const std::bad_variant_access& e) {
		std::cerr << "[AUTOPILOT] Error reading aff_interacting: " << e.what() << std::endl;
	}
}

void SpecificWorker::check_recording_handshake()
{
	// Only if waiting for handshake
	if (handshake_waiting_mission_id == 0) {
		return;
	}
	
	// Check timeout first
	handshake_timeout_cycles++;
	if (handshake_timeout_cycles > HANDSHAKE_TIMEOUT_CYCLES) {
		std::cerr << "[HANDSHAKE_TIMEOUT] episodic_memory did not respond within " 
		          << HANDSHAKE_TIMEOUT_CYCLES << " cycles (~500ms)" << std::endl;
		std::cerr << "[HANDSHAKE_TIMEOUT] Aborting mission activation. Cleaning up TARGET edge." << std::endl;
		
		// Rollback: remove TARGET edge
		delete_mission_target_edge(handshake_waiting_mission_id);
		
		// Reset state
		handshake_waiting_mission_id = 0;
		handshake_timeout_cycles = 0;
		return;
	}
	
	// Check two-phase handshake: initialization_started=true AND recording=true
	try {
		auto mission_opt = mission_graph->get_node(handshake_waiting_mission_id);
		if (!mission_opt.has_value()) {
			std::cerr << "[HANDSHAKE_ERROR] Mission node not found" << std::endl;
			handshake_waiting_mission_id = 0;
			handshake_timeout_cycles = 0;
			return;
		}
		
		auto mission = mission_opt.value();
		
		// Phase 1: Check if episodic_memory detected TARGET and began initialization
		auto init_started_iter = mission.attrs().find("initialization_started");
		bool init_started = false;
		if (init_started_iter != mission.attrs().end()) {
			try {
				init_started = std::get<bool>(init_started_iter->second.value());
			} catch (const std::bad_variant_access& e) {
				std::cerr << "[HANDSHAKE_ERROR] initialization_started type mismatch: " << e.what() << std::endl;
			}
		}
		
		// Phase 2: Check if episodic_memory completed initialization and is RECORDING
		auto recording_iter = mission.attrs().find("recording");
		bool recording = false;
		if (recording_iter != mission.attrs().end()) {
			try {
				recording = std::get<bool>(recording_iter->second.value());
			} catch (const std::bad_variant_access& e) {
				std::cerr << "[HANDSHAKE_ERROR] recording type mismatch: " << e.what() << std::endl;
			}
		}
		
		// Both phases must be complete for handshake success
		if (init_started && recording) {
			// ✓ Handshake successful: episodic_memory detected TARGET and completed initialization
			std::cout << "[HANDSHAKE] ✓ Phase-1: initialization_started=true" << std::endl;
			std::cout << "[HANDSHAKE] ✓ Phase-2: recording=true (episodic_memory ready)" << std::endl;
			
			uint64_t mission_id = handshake_waiting_mission_id;
			
			// Note: mission status was already set to "running" in the activation callback
			// Just activate affordances here - agents will react immediately
			on_startMission_clicked();
			
			// Reset handshake state
			handshake_waiting_mission_id = 0;
			handshake_timeout_cycles = 0;
			
			std::cout << "[HANDSHAKE] ✓ Mission activation sequence completed. Affordances activated." << std::endl;
			return;
		}
		
		// Log progress for debugging
		if (init_started && !recording) {
			std::cout << "[HANDSHAKE] Phase-1 OK, Phase-2 in progress... (cycle " 
			          << handshake_timeout_cycles << "/" << HANDSHAKE_TIMEOUT_CYCLES << ")" << std::endl;
		} else if (!init_started) {
			std::cout << "[HANDSHAKE] Waiting for Phase-1 (initialization_started)... (cycle " 
			          << handshake_timeout_cycles << "/" << HANDSHAKE_TIMEOUT_CYCLES << ")" << std::endl;
		}
		
		// If both phases not complete and not timeout yet, keep waiting (next compute cycle)
		
	} catch (const std::exception& e) {
		std::cerr << "[HANDSHAKE_ERROR] Exception: " << e.what() << std::endl;
		handshake_waiting_mission_id = 0;
		handshake_timeout_cycles = 0;
	}
}

void SpecificWorker::on_aff_interacting_false()
{
	std::cout << "\n[AUTOPILOT] ⚠ aff_interacting changed to FALSE!" << std::endl;
	
	// Check if autopilot mode or manual mode
	if (autopilot_enabled && autopilot_state == AutopilotState::RUNNING) {
		// === AUTOPILOT MODE: Call new completion handler ===
		std::cout << "[AUTOPILOT] Mode: AUTOPILOT - transitioning to mission completion..." << std::endl;
		autopilot_on_mission_complete();
	} else {
		// === MANUAL MODE: Legacy behavior ===
		std::cout << "[AUTOPILOT] Mode: MANUAL - legacy stop behavior..." << std::endl;
		
		// Calculate elapsed time before stopping
		auto now = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration<float>(now - mission_start_time).count();
		mission_accumulated_time += elapsed;
		
		int stopped_row = active_mission_row;
		
		// Stop current mission through scheduler
		if (mission_row_to_node_id.count(stopped_row) > 0)
		{
			uint64_t stopped_mission_id = mission_row_to_node_id[stopped_row];
			mission_scheduler.stopMission(stopped_mission_id);
			
			// Update UI and graph
			model->setMissionStatus(stopped_row, MissionStatus::STOPPED);
			model->setMissionElapsedTime(stopped_row, mission_accumulated_time);
			update_mission_status_episodic(stopped_mission_id, "stopped");
			delete_mission_target_edge(stopped_mission_id);
			
			// Save elapsed_time to graph
			try {
				auto mission_opt = mission_graph->get_node(stopped_mission_id);
				if (mission_opt.has_value())
				{
					auto mission = mission_opt.value();
					DSR::Attribute elapsed_attr;
					elapsed_attr.value(mission_accumulated_time);
					mission.attrs()["elapsed_time"] = elapsed_attr;
					mission_graph->update_node(mission);
				}
			} catch (const std::exception& e) {
				std::cerr << "Error saving elapsed time: " << e.what() << std::endl;
			}
			
			QModelIndex stopped_idx = model->index(stopped_row, 0);
			mission_controller_ui.mission_list->update(stopped_idx);
		}
		
		// Reset mission tracking
		active_mission_row = -1;
		follow_person_active = false;
		
		std::cout << "[AUTOPILOT] ✓ follow_person mission stopped (MANUAL)" << std::endl;
	}
}

// ============================= NEW AUTOPILOT STATE MACHINE METHODS =============================

void SpecificWorker::autopilot_step()
{
	// Debug: log state machine status every 100 cycles
	static int debug_cycle_counter = 0;
	if (++debug_cycle_counter % 100 == 0) {
		std::cout << "[AUTOPILOT-DEBUG] State=" << static_cast<int>(autopilot_state) 
		          << " mission_id=" << current_active_mission_id 
		          << " enabled=" << autopilot_enabled << std::endl;
	}
	
	switch (autopilot_state)
	{
		case AutopilotState::IDLE:
			// State machine not initialized - should not happen if autopilot_enabled=true
			break;
		
		case AutopilotState::SELECTING_NEXT:
			autopilot_select_next_mission();
			break;
		
		case AutopilotState::WAITING_AFFORDANCE:
		{
			// Check if mission was completed before affordance (manual UI click)
			if (current_active_mission_id > 0) {
				try {
					auto mission_opt = mission_graph->get_node(current_active_mission_id);
					if (mission_opt.has_value()) {
						auto status_attr = mission_opt.value().attrs().find("status");
						if (status_attr != mission_opt.value().attrs().end()) {
							try {
								std::string status = std::get<std::string>(status_attr->second.value());
								if (status == "completed") {
									// Mission was completed before entering RUNNING (manual click)
									std::cout << "[AUTOPILOT] ⚠ Mission completed while in WAITING state. Synchronizing..." << std::endl;
									autopilot_on_mission_complete();
									break;
								}
							} catch (const std::bad_variant_access&) {}
						}
					}
				} catch (const std::exception&) {}
			}
			// Normal flow: check affordance
			check_affordance_and_activate();
			break;
		}
		
		case AutopilotState::RUNNING:
		{
			std::cout << "[AUTOPILOT-RUNNING-CYCLE] Entering RUNNING case, mission_id=" << current_active_mission_id << std::endl;
			
			// Check if mission was completed manually via UI
			if (current_active_mission_id > 0) {
				std::cout << "[AUTOPILOT-RUNNING-CYCLE] mission_id > 0, trying to get node..." << std::endl;
				try {
					auto mission_opt = mission_graph->get_node(current_active_mission_id);
					if (!mission_opt.has_value()) {
						// Mission node deleted or not found
						std::cout << "[AUTOPILOT-RUNNING-CYCLE] Mission node NOT found" << std::endl;
						autopilot_on_mission_complete();
					} else {
						std::cout << "[AUTOPILOT-RUNNING-CYCLE] Mission node found. Checking status..." << std::endl;
						auto status_attr = mission_opt.value().attrs().find("status");
						if (status_attr != mission_opt.value().attrs().end()) {
							try {
								std::string status = std::get<std::string>(status_attr->second.value());
								std::cout << "[AUTOPILOT-RUNNING-CYCLE] Status value: '" << status << "'" << std::endl;
								if (status == "completed") {
									std::cout << "[AUTOPILOT] ⚠ Mission marked as completed. Synchronizing..." << std::endl;
									autopilot_on_mission_complete();
								} else {
									monitor_aff_interacting_state();
								}
							} catch (const std::bad_variant_access& e) {
								std::cerr << "[AUTOPILOT-ERR] bad_variant_access: " << e.what() << std::endl;
								monitor_aff_interacting_state();
							}
						} else {
							std::cout << "[AUTOPILOT-RUNNING-CYCLE] Status attribute NOT found" << std::endl;
							monitor_aff_interacting_state();
						}
					}
				} catch (const std::exception& e) {
					std::cerr << "[AUTOPILOT-ERR] get_node failed: " << e.what() << std::endl;
					monitor_aff_interacting_state();
				}
			} else {
				std::cout << "[AUTOPILOT-RUNNING-CYCLE] mission_id is 0! Should not happen." << std::endl;
				autopilot_state = AutopilotState::SELECTING_NEXT;
			}
			break;
		}
		
		case AutopilotState::COMPLETED:
			// Transition to next select
			autopilot_state = AutopilotState::SELECTING_NEXT;
			break;
	}
}

void SpecificWorker::autopilot_select_next_mission()
{
	if (!has_pending_missions()) {
		// No pending missions
		idle_cycles_count++;
		
		if (idle_cycles_count > MAX_IDLE_CYCLES) {
			// Timeout: create follow_person_default as fallback
			std::cout << "[AUTOPILOT] No pending missions for " << idle_cycles_count 
			          << " cycles. Creating follow_person fallback..." << std::endl;
			
			create_or_check_follow_person_mission();
			
			// Reset counter to check sooner next time (5 cycles instead of 100)
			// This prevents repeatedly creating fallbacks while waiting for scheduler sync
			idle_cycles_count = MAX_IDLE_CYCLES - 5;
		}
		return;
	}
	
	// There are pending missions - select next by priority
	auto next_mission_opt = mission_scheduler.selectNextMission();
	
	if (!next_mission_opt.has_value()) {
		std::cerr << "[AUTOPILOT] ERROR: has_pending_missions returned true but selectNextMission returned None" << std::endl;
		return;
	}
	
	uint64_t next_mission_id = next_mission_opt.value();
	idle_cycles_count = 0;  // Reset idle counter since we found a mission
	
	// Get mission info from model to find the type ("Follow Person", "Search Problem Cause", etc)
	std::string mission_type = get_mission_type_from_id(next_mission_id);
	if (mission_type.empty()) {
		std::cerr << "[AUTOPILOT] ERROR: Could not determine mission type for ID " << next_mission_id << std::endl;
		return;
	}
	
	std::cout << "[AUTOPILOT] Selected mission_id=" << next_mission_id 
	          << " type=" << mission_type << std::endl;
	
	// Store for tracking
	current_active_mission_id = next_mission_id;
	current_active_mission_type = mission_type;
	
	// Activate mission: create TARGET edge and start handshake
	autopilot_activate_mission(next_mission_id, mission_type);
	
	// Transition to waiting for affordance/handshake
	autopilot_state = AutopilotState::WAITING_AFFORDANCE;
}

void SpecificWorker::autopilot_activate_mission(uint64_t mission_id, const std::string& type)
{
	std::cout << "[AUTOPILOT] Activating mission id=" << mission_id << " type=" << type << std::endl;
	
	// Activate through scheduler (handles preemption)
	auto result = mission_scheduler.activateMission(mission_id);
	if (!result.success) {
		std::cerr << "[AUTOPILOT] ERROR: Failed to activate mission" << std::endl;
		autopilot_state = AutopilotState::SELECTING_NEXT;
		return;
	}
	
	// Update DSR status to "running"
	update_mission_status_episodic(mission_id, "running");
	
	// Create TARGET edge for episodic_memory
	create_mission_target_edge(mission_id);
	
	// Start handshake with episodic_memory
	handshake_waiting_mission_id = mission_id;
	handshake_timeout_cycles = 0;
	
	std::cout << "[AUTOPILOT] Created TARGET edge, waiting for episodic_memory handshake..." << std::endl;
}

void SpecificWorker::autopilot_on_mission_complete()
{
	std::cout << "\n[AUTOPILOT] Mission completed: id=" << current_active_mission_id << std::endl;
	
	// Mark as completed in scheduler
	mission_scheduler.completeMission(current_active_mission_id);
	
	// Update DSR status
	update_mission_status_episodic(current_active_mission_id, "completed");
	
	// Calculate and save elapsed time
	auto now = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration<float>(now - mission_start_time).count();
	mission_accumulated_time += elapsed;
	
	try {
		auto mission_opt = mission_graph->get_node(current_active_mission_id);
		if (mission_opt.has_value())
		{
			auto mission = mission_opt.value();
			DSR::Attribute elapsed_attr;
			elapsed_attr.value(mission_accumulated_time);
			mission.attrs()["elapsed_time"] = elapsed_attr;
			mission_graph->update_node(mission);
		}
	} catch (const std::exception& e) {
		std::cerr << "[AUTOPILOT] Error saving elapsed time: " << e.what() << std::endl;
	}
	
	// Cleanup TARGET edge
	delete_mission_target_edge(current_active_mission_id);
	
	// Reset current mission tracking
	current_active_mission_id = 0;
	current_active_mission_type = "";
	follow_person_active = false;
	
	std::cout << "[AUTOPILOT] Mission cleanup complete. Ready for next mission." << std::endl;
	
	// Transition to select next mission
	autopilot_state = AutopilotState::SELECTING_NEXT;
}

bool SpecificWorker::has_pending_missions() const
{
	// Query scheduler for pending missions (priority order)
	auto pending = mission_scheduler.selectNextMission();
	return pending.has_value();
}

// Helper: Get mission type string ("Follow Person", "Search Problem Cause") from mission ID
std::string SpecificWorker::get_mission_type_from_id(uint64_t mission_id) const
{
	// Reverse lookup: find row from mission_id
	for (const auto& [row, id] : mission_row_to_node_id) {
		if (id == mission_id) {
			// Found the row, get mission type from model
			Mission mission = model->getMission(row);
			return mission.type.toStdString();  // Convert QString to std::string
		}
	}
	return "";  // Not found
}

