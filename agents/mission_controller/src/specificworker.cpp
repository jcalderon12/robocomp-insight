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
#include <iomanip>
#include <sstream>
#include <ctime>
#include <dirent.h>
#include <sys/stat.h>
#include <random>

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

	// Agregar el widget personalizado en el dock de la ventana episodic
	if (!graph_viewers.empty())
		graph_viewers.at("episodic")->add_custom_widget_to_dock("Mission Controller", &mission_controller_widget);

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
			auto elapsed = std::chrono::duration<float>(now - mission_start_time).count();
			mission_accumulated_time += elapsed;
			
			model->setMissionStatus(row, MissionStatus::STOPPED);
			
			// Update mission status to "stopped" and delete TARGET edge in episodic graph
			if (mission_row_to_node_id.count(row) > 0) {
				uint64_t mission_id = mission_row_to_node_id[row];
				update_mission_status_episodic(mission_id, "stopped");
				delete_mission_target_edge(mission_id);
			}
			
			on_stopMission_clicked();
		}
		else if (m.status != MissionStatus::COMPLETED)
		{
			active_mission_row = row;
			mission_start_time = std::chrono::steady_clock::now();
			model->setMissionStatus(row, MissionStatus::RUNNING);
			
			// Update mission status to "running" and create TARGET edge in episodic graph
			if (mission_row_to_node_id.count(row) > 0) {
				uint64_t mission_id = mission_row_to_node_id[row];
				update_mission_status_episodic(mission_id, "running");
				create_mission_target_edge(mission_id);
			}
			
			on_startMission_clicked();
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
		
		// NO CARGAR CAMBIOS AQUÍ - El episodic_memory es responsable de gestionar los cambios
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
			
			if (customName.isEmpty())
			{
				customName = missionType;
			}
			
			Mission newMission{customName, missionType};
			int row = model->rowCount();  // Get future row index
			model->addMission(newMission);
			
			// Insert mission node in episodic graph with status="pending"
			insert_mission_node_episodic(customName.toStdString(), row);
			
			on_setMission_clicked();
		}
	});

	// Configurar timer para actualizar tiempo en tiempo real
	mission_timer = new QTimer(this);
	connect(mission_timer, &QTimer::timeout, this, &SpecificWorker::updateMissionTime);
	mission_timer->start(100);  // Actualizar cada 100ms

	// === HISTORIC DEBUGGER SETUP ===
	historic_debugger_ui.setupUi(&historic_debugger_widget);
	
	if (!graph_viewers.empty())
		graph_viewers.at("debugger")->add_custom_widget_to_dock("Historic debugger", &historic_debugger_widget);
	
	// Connect debugger scrollbars and search
	connect(historic_debugger_ui.local_changes_scroll_bar, &QScrollBar::valueChanged, this, &SpecificWorker::local_changes_management);
	connect(historic_debugger_ui.global_changes_scroll_bar, &QScrollBar::valueChanged, this, &SpecificWorker::global_changes_management);
	connect(historic_debugger_ui.time_input, &QLineEdit::returnPressed, this, &SpecificWorker::on_time_search);
	
	// Connect load_in_debugger button if it exists
	if (mission_controller_ui.load_in_debugger_button)
	{
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
	
	std::cout << "Historic debugger widget initialized" << std::endl;

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
	
	// We need to find and load the saved mission file for this mission
	// Since missions are saved with a timestamp, we need to match by mission name or last saved
	// For now, load the most recent mission file
	std::string most_recent_mission = "";
	time_t most_recent_time = 0;
	
	DIR* dir = opendir("./completed_missions");
	if (dir != nullptr)
	{
		struct dirent* entry;
		while ((entry = readdir(dir)) != nullptr)
		{
			std::string filename = entry->d_name;
			if (filename.find("mission_") == 0 && filename.find(".txt") != std::string::npos)
			{
				std::string filepath = "./completed_missions/" + filename;
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
	
	if (!most_recent_mission.empty())
	{
		// NO CARGAR CAMBIOS AQUÍ - El episodic_memory es responsable de gestionar los cambios
		std::cout << "Mission completed and stored. episodic_memory will manage the changes." << std::endl;
	}
	else
	{
		QMessageBox::warning(&mission_controller_widget, "Error", "No completed missions found to load");
	}
}

// ===== METHODS FOR MISSION MANAGEMENT IN EPISODIC GRAPH =====

std::optional<uint64_t> SpecificWorker::insert_mission_node_episodic(const std::string &mission_name, int row) {
	try {
		DSR::Node mission_node;
		mission_node.type("intention");
		mission_node.name(mission_name);
		mission_node.agent_id(agent_id);
		
		// Add status attribute (initially "pending")
		DSR::Attribute status_attr;
		status_attr.value("pending");
		mission_node.attrs()["status"] = status_attr;
		
		auto mission_id_opt = mission_graph->insert_node(mission_node);
		
		if (mission_id_opt.has_value()) {
			uint64_t mission_id = mission_id_opt.value();
			mission_row_to_node_id[row] = mission_id;
			std::cout << "Mission node created in episodic graph: " << mission_name << " (id: " << mission_id << ")" << std::endl;
			
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
			
			return mission_id;
		} else {
			std::cerr << "Failed to insert mission node: " << mission_name << std::endl;
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
			std::cerr << "Mission node not found: " << mission_id << std::endl;
			return;
		}
		
		auto mission = mission_opt.value();
		DSR::Attribute status_attr;
		status_attr.value(status);
		mission.attrs()["status"] = status_attr;
		
		mission_graph->update_node(mission);
		std::cout << "Mission status updated to: " << status << std::endl;
	} catch (const std::exception& e) {
		std::cerr << "Exception updating mission status: " << e.what() << std::endl;
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
		std::cout << "Edge TARGET created from robot to mission (id: " << mission_id << ")" << std::endl;
	} catch (const std::exception& e) {
		std::cerr << "Exception creating TARGET edge: " << e.what() << std::endl;
	}
}

void SpecificWorker::delete_mission_target_edge(uint64_t mission_id) {
	try {
		auto robot_opt = mission_graph->get_node("robot");
		if (!robot_opt.has_value()) {
			std::cerr << "Robot node not found in episodic graph" << std::endl;
			return;
		}
		
		mission_graph->delete_edge(robot_opt.value().id(), mission_id, "TARGET");
		std::cout << "Edge TARGET deleted from robot to mission (id: " << mission_id << ")" << std::endl;
	} catch (const std::exception& e) {
		std::cerr << "Exception deleting TARGET edge: " << e.what() << std::endl;
	}
}

std::optional<uint64_t> SpecificWorker::find_mission_node_by_name(const std::string &mission_name) {
	try {
		// Search for intention nodes with matching name
		auto nodes = mission_graph->get_nodes_by_type("intention");
		
		for (const auto &node : nodes) {
			if (node.name() == mission_name) {
				return node.id();
			}
		}
		
		return std::nullopt;
	} catch (const std::exception& e) {
		std::cerr << "Exception finding mission node: " << e.what() << std::endl;
		return std::nullopt;
	}
}


