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

// Robot maximum speeds
static constexpr float WEBOTS_MAX_LINEAR_SPEED  = 1.5f; //meters per second
static constexpr float WEBOTS_MAX_ANGULAR_SPEED = 4.03f; //radians per second 


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <Eigen/Geometry>

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
	 * \brief Method to check if there is an active affordance in the DSR graph. 
	 * An active affordance is an affordance node that comes from the person node target and has the attribute aff_interacting_att to true.
	 * \return Return true if there is an active affordance. false otherwise.
	 */
	bool check_affordance_active();

	/**
	 * \brief Calculate the relative position between the robot and the person
	 * \return Return a vector with the relative position (x, y, z)
	 */
	std::vector<float> get_person_relative_position();

	/**
	 * \brief Per-axis median of the valid points of a cloud (finite, ahead of the camera).
	 * Zeros if none is valid.
	 */
	std::vector<float> median_point(const RoboCompImageSegmentation::PointCloud& cloud);

	/**
	 * \brief Person memory: each detection is stored in the root frame and, with no detection,
	 * the relative position is rebuilt from that memory and the current robot pose instead of
	 * returning zeros. Publishes perceived=true/false on the person node.
	 * \param held True if a detection was withheld by confirm_person_jump(): memory is used but
	 * perceived is left unchanged.
	 */
	std::vector<float> apply_person_memory(const std::vector<float>& detected, bool held = false);

	/**
	 * \brief Jump filter: a detection farther than large_jump_threshold from the last published
	 * position is accepted only after required_large_jump_confirmations consistent samples.
	 * \return True if the detection can be used
	 */
	bool confirm_person_jump(const std::vector<float>& relative_position);

	/**
	 * \brief Robot pose from the root->robot RT written by concept_robot: x, y (meters) and the
	 * quaternion yaw (radians). False if the RT is missing.
	 */
	bool get_robot_pose_2d(float &x, float &y, float &yaw);

	/**
	 * \brief Writes perceived on the person node, only when it changes.
	 */
	void set_person_perceived(bool perceived);

	/**
	 * \brief Compare whether two vectors are too similar to avoid publishing the same data twice.
	 * \return Return true if the vector are not to similar
	 */
	bool has_significant_change(const std::vector<float>& a,const std::vector<float>& b,double atol=0.001);

	/**
	 * \brief Update the relative position between the robot and the person in the DSR
	 * \param relative_position The relative position between the robot and the person
	 * \return True if the position was actually updated in DSR
	 */
	bool update_relative_position_to_person(const std::vector<float>& relative_position, std::uint64_t now_ms);

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
	bool print_extra_info = configLoader.get<bool>("print_extra_info");

	bool simulated = configLoader.get<bool>("Simulated");

	std::vector<float> last_relative_pose = {0.0f, 0.0f, 0.0f}; // {x, y, z}

	// ---- Memoria de la persona (apply_person_memory()) ----
	// Un vector nulo es "no la veo", no una posicion: es lo que devuelve la segmentacion sin persona.
	static constexpr float PERSON_NULL_DISTANCE = 1e-3f;    // metros
	// Tope de la memoria. Generoso a proposito: el giro de take_photos dura minuto y medio y la
	// persona se ve en algun paso de la vuelta, pero puede quedar a la espalda al terminar.
	static constexpr float PERSON_MEMORY_MAX_AGE = 120.f;   // segundos
	bool  person_memory_valid = false;
	float person_memory_x = 0.f, person_memory_y = 0.f, person_memory_z = 0.f;  // marco root, metros
	std::chrono::steady_clock::time_point person_memory_stamp;
	int   person_perceived_published = -1;   // -1 nunca publicado; 0/1 el ultimo valor
	bool  person_memory_expired_warned = false;
	std::vector<float> pending_relative_position = {0.0f, 0.0f, 0.0f};
	int consecutive_large_jump_confirmations = 0;
	int required_large_jump_confirmations = 3;
	float large_jump_threshold = 1.0f; // metros
	float large_jump_candidate_tolerance = 0.2f; // metros

signals:
	//void customSignal();
};

#endif
