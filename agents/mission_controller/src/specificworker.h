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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "historic_manager.h"
#include "DSRDecoder.h"
#include "DSRTypeTrait.h"
#include <fstream>
#include "ui_mission_controller.h"
#include "MissionModel.h"
#include "MissionDelegate.h"
#include "ui_add_mission_dialog.h"
#include "ui_historic_debugger.h"
#include "MissionScheduler.h"
#include <chrono>

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();

	/**
	 * \brief Retrieves a list of available missions.
	 * \return A vector of strings representing the available missions.
	 */
	std::vector<std::string> getAvailableMissions() const
	{
		return {
			"Follow Person",
			"Search Problem Cause"
		};
	}

public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

	/**
	 * \brief Slot triggered when the "Stop Mission" button is clicked.
	 */
	void on_stopMission_clicked();
	/**
	 * \brief Slot triggered when the "Set Mission" button is clicked.
	 */
	void on_setMission_clicked();
	/**
	 * \brief Slot triggered when the "Start Mission" button is clicked.
	 */
	void on_startMission_clicked();
	
	// Historic debugger slots
	void local_changes_management(int value);
	void global_changes_management(int value);
	void on_time_search();
	void update_local_scrollbar(size_t keyframe_idx);
	void load_mission_in_debugger(int row);

	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names);
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
	void modify_edge_attrs_slot(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names){};
	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};     
private:

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	std::shared_ptr<DSR::DSRGraph> mission_graph;

	Ui::mission_controller mission_controller_ui;
	QWidget mission_controller_widget;

	MissionModel *model;
    MissionDelegate *delegate;

	int active_mission_row = -1;
	std::chrono::steady_clock::time_point mission_start_time;
	float mission_accumulated_time = 0;
	QTimer *mission_timer;

	// Map: model row index -> mission node id in episodic graph
	std::map<int, uint64_t> mission_row_to_node_id;
	
	// Mission scheduler for priority-based mission management
	MissionScheduler mission_scheduler;
	
	// Mission layout control for episodic graph visualization
	int missions_in_current_row = 0;  // Counter for missions in current row (max 5)
	float current_y_offset = 200.0f;   // Y offset from robot position (starts at 200)

	void updateMissionTime();

	// Historic debugger widget
	Ui::historic_debugger historic_debugger_ui;
	QWidget historic_debugger_widget;

	// === DEBUGGER VARIABLES ===
	std::shared_ptr<DSR::DSRGraph> historic_graph;
	std::unique_ptr<HistoricManager> historic_manager;  // Viewer created automatically by config
	
	std::map<uint64_t, std::string> changes_map;  // timestamp -> dsr_data
	std::map<uint64_t, DSREvent> decoded_data;    // timestamp -> decoded event
	
	int historic_value = 0;  // Current position in historic
	
	// Methods for debugger
	void load_mission_changes(const std::string filename);
	void display_debugger_graph();

	// Methods for mission management in episodic graph
	std::optional<uint64_t> insert_mission_node_episodic(const std::string &mission_name, int row, int priority);
	void update_mission_status_episodic(uint64_t mission_id, const std::string &status);
	void create_mission_target_edge(uint64_t mission_id);
	void delete_mission_target_edge(uint64_t mission_id);
	std::optional<uint64_t> find_mission_node_by_name(const std::string &mission_name);

signals:
	//void customSignal();
};

#endif
