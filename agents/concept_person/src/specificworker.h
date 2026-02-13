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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#define WEBOTS_MAX_LINEAR_SPEED 1.5 //meters per second
#define WEBOTS_MAX_ANGULAR_SPEED 4.03 //radians per second 


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>

enum class States
{
	IDLE,
	ASSIGNED,
	FOLLOWME,
	SUCCESS,
	FAILED,
	STOPPED
};


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

	States agentState = States::IDLE;

	std::string person_def = "HUMAN_2";
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
	 * \brief Checks the current state of the mission in the DSR.
	 * \return A boolean indicating whether the target is assigned.
	 */
	bool check_target_assigned();

	/**
	 * \brief Checks if an affordance already exists for the assigned target.
	 * \return A boolean indicating whether the affordance exists.
	 */
	bool check_affordance_assigned();
	
	/**
	 * \brief Creates an affordance for the assigned target.	
	 */
	void create_affordance();

	/**
	 * \brief Checks if an affordance has been accepted for the assigned target.
	 * \return A boolean indicating whether the affordance is accepted.
	 */
	bool check_affordance_accepted();

	/**
	 * \brief Follows the assigned person.
	 * \return A boolean indicating if the robot should follow the robot or stop.
	 */
	bool follow_person(std::vector<float> distance);

	/**
	 * \brief Calculate the cartesian distance between the robot and the person
	 * \return Return a vector with the cartesian distance
	 */
	std::vector<float> get_distance_to_person();

	/**
	 * \brief Update the distance between the robot and the person in the DSR
	 * \param distance The distance between the robot and the person
	 */
	void update_distance_to_person(std::vector<float> distance);

	/**
	 * \brief Calculate the speed that the robot must have to reach the person
	 * \param distance The distance between the robot and the person
	 * \return The speed calculated like a proporcional controller
	 */
	float calculate_linear_speed_from_distance(std::vector<float> distance);

private:

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	/**
	 * \brief Flag to indicate if want many info
	 */
	bool print_extra_info = false;

signals:
	//void customSignal();
};

#endif
