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
	\brief Keeps a live robot->bump RT edge, the same way concept_person does for "person"
	and concept_bottle for "bottle". Unlike those, it manages no affordance/state machine:
	semantic creates the "bump" node and mission_controller creates the TARGET edge; this
	agent's only job is geometry, while "bump" exists in the graph.
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <Eigen/Geometry>

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

	void VisualElementsPub_setVisualObjects(RoboCompVisualElementsPub::TData data);

	std::string bump_def = "BUMP";   // Webots DEF label of the bump Solid (see SimpleWorld_Bump.wbt)
	std::string robot_def = "shadow";
	std::unique_ptr<DSR::RT_API> rt;

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

	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
	void modify_edge_attrs_slot(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names){};
	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};

	/**
	 * \brief Calculate the relative position between the robot and the bump, in the
	 * robot's local frame (meters). Simulated: Webots ground truth for both robot and
	 * bump (DEF lookup), same 2D-yaw projection concept_person uses. Real: TODO, needs
	 * a "bump" label from ImageSegmentation (not trained yet).
	 * \return {x, y, z} in the robot's local frame.
	 */
	std::vector<float> get_bump_relative_position();

	/**
	 * \brief Compare whether two vectors are too similar to avoid publishing the same data twice.
	 * \return Return true if the vectors are NOT too similar (i.e. there IS a significant change)
	 */
	bool has_significant_change(const std::vector<float>& a,const std::vector<float>& b,double atol=0.001);

	/**
	 * \brief Update the relative position between the robot and the bump in the DSR
	 * (robot->bump RT edge). Debounces large single-sample jumps the same way
	 * concept_person does, requiring several consecutive confirmations before accepting
	 * a big position change (protects against a single noisy pose reading).
	 * \param relative_position The relative position between the robot and the bump (m)
	 * \return True if the position was actually written to the DSR
	 */
	bool update_relative_position_to_bump(const std::vector<float>& relative_position, std::uint64_t now_ms);

private:

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	/**
	 * \brief Flag to indicate if want many info
	 */
	bool print_extra_info = false;

	bool simulated = configLoader.get<bool>("Simulated");

	std::vector<float> last_relative_pose = {0.0f, 0.0f, 0.0f}; // {x, y, z}, meters
	std::vector<float> pending_relative_position = {0.0f, 0.0f, 0.0f};
	bool has_published_once = false; // force the very first RT write, don't rely on read noise
	int consecutive_large_jump_confirmations = 0;
	int required_large_jump_confirmations = 3;
	float large_jump_threshold = 1.0f;            // meters (the bump is static: a real jump this big means bad data, not motion)
	float large_jump_candidate_tolerance = 0.2f;   // meters

signals:
	//void customSignal();
};

#endif
