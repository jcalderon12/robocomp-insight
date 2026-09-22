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
#include <vector>
#include <cmath>
#include <numbers>
#include <array>
#include <string>
#include <opencv2/opencv.hpp>

// Robot maximum speeds
static constexpr float WEBOTS_MAX_LINEAR_SPEED  = 1.5f; //meters per second
static constexpr float WEBOTS_MAX_ANGULAR_SPEED = 4.03f; //radians per second 


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

	void FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose);


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
	 * \brief Obtain target velocities from the robot node on the DSR graph.
	 * \return A vector of two elements, the forward velocity and the angular velocity.
	 */
	std::vector<float> getVelocitiesFromDSR();

	/**
	 * \brief Create the imu node or updated it in the DSR graph. 
	 * If the IMU measurement its the same from the last call of this method, it dont do nothing 
	 */
	void update_or_create_imu_node();

	/**
	 * \brief Compare whether two vectors are too similar to avoid publishing the same data twice.
	 * \return Return true if the vector are not to similar
	 */
	bool has_significant_change(const std::vector<float>& a,const std::vector<float>& b,double atol=0.001);

	/**
	 * \brief This method calculates the desired linear and angular velocities to follow a target while maintaining a certain distance. 
	 * It uses a proportional controller to compute the velocities based on the distance and angle to the target, and then sends these velocities to the DSR graph. 
	 * If the target is too close, it stops the robot.
	 * \param max_forward_speed_factor: Maximum forward speed factor to apply to the robot (between 0 and 1).
	 * \param max_angular_speed_factor: Maximum angular speed factor to apply to the robot (between 0 and 1).
	 * \param desired_distance: Desired distance to the target in meters.
	 * 
	 */
	void follow_target(float max_forward_speed_factor = 0.6f, float max_angular_speed_factor = 0.6f, float desired_distance = 0.5f);

	/**
	 * \brief This method calculates the robot position in the actual room and update the DSR graph with this information. 
	 * It uses the auto_localization method to get the robot position and orientation, and then updates the corresponding attributes in the DSR graph. 
	 * If the robot node does not exist in the DSR graph, it creates it.
	 * \return The robot pose in the format {x, y, z, qx, qy, qz, qw}.
	 */
	std::vector<float> auto_localization();

	/**
	 * \brief For a TARGET pointing to a node with a static global "problem_position" (e.g.
	 * "bump"), computes and writes a live RT robot->target edge each cycle, so follow_target()
	 * has something to read. No-op if the target has no "problem_position" (e.g. "person",
	 * whose RT robot->target is already maintained live by concept_person).
	 */
	void update_static_target_rt();

	/**
	 * \brief Method to check if there is an active affordance in the DSR graph.
	 * An active affordance is an affordance node that comes from the person node target and has the attribute aff_interacting_att to true.
	 * \return Return true if there is an active affordance. false otherwise.
	 */
	bool check_affordance_active();

	/**
	 * \brief Method to set the robot speed in zero.
	 */
	void stop_robot();

	/**
	 * \brief True when the current TARGET points at the "bump" node, i.e. this is a photo
	 * session (orbit_target()) rather than a plain follow (follow_target()).
	 */
	bool is_orbit_target();

	/**
	 * \brief Drives the robot to Photo_distance from the "bump" target, then orbits it in
	 * Orbit_points evenly-spaced stops. At each stop it faces the bump and takes
	 * Photos_per_point "con_bache" photos (panning by Photo_angular_step between shots so the
	 * bump never leaves frame), then turns to face away (same rotate-by-Photo_angular_step
	 * primitive, just repeated further, never a single big jump) and takes Photos_per_point
	 * "sin_bache" photos. Finishes by clearing aff_interacting on the photo affordance, which
	 * mission_controller watches to complete the "Take Photos" mission.
	 */
	void orbit_target();

	/**
	 * \brief Core of follow_target()/orbit_target(): P/D-controlled drive toward an arbitrary
	 * point already expressed in the robot's current local frame (meters), stopping once within
	 * desired_distance of it. Writes robot_ref_adv_speed/robot_ref_rot_speed to the DSR.
	 */
	void drive_to_local_point(float x, float y, float desired_distance,
		float max_forward_speed_factor, float max_angular_speed_factor);

	/**
	 * \brief Rotates in place (linear speed 0) to reduce heading_error (radians), P-controlled
	 * and velocity-clamped like drive_to_local_point -- used for both the small panning steps
	 * between photos and the big con-bache/sin-bache turn, so neither is a single abrupt jump.
	 */
	void rotate_in_place(float heading_error, float max_angular_speed_factor = 0.6f);

	/**
	 * \brief Drives straight ahead (angular speed 0, no steering) to reduce distance_to_target
	 * toward desired_distance, P-controlled and velocity-clamped like drive_to_local_point.
	 * Used by orbit_target()'s TRAVEL stage's "look then go" sequence: rotate_in_place() first
	 * to face the waypoint, only then drive_straight() -- deliberately simple, revisit later.
	 */
	void drive_straight(float distance_to_target, float desired_distance, float max_forward_speed_factor = 1.0f);

	/**
	 * \brief Grabs one Camera360RGB frame and saves it as a .ppm under Photo_save_dir/<label>/.
	 * distance_to_bump (meters) is only used for the debug log, not the capture itself.
	 */
	void take_photo(const std::string& label, float distance_to_bump);

	/**
	 * \brief Sets aff_interacting=false on the affordance reached via TARGET->has_intention
	 * (mirrors the check in check_affordance_active()), signalling mission_controller that the
	 * photo session is done.
	 */
	void complete_photo_affordance();

	/**
	 * \brief Resets orbit_target()'s progress (stage/waypoint/shot index/captured bearing) so
	 * the next "Take Photos" mission starts from scratch.
	 */
	void reset_orbit_state();

	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
	void modify_edge_attrs_slot(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names){};
	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};     
private:

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	std::vector<float> last_acceleration_measurement;
	std::vector<float> last_angular_velocity_measurement;

	std::vector<float> last_velocities_readed;
	std::vector<float> last_robot_pose;

	std::vector<float> last_odometry;

	static constexpr float HALF_PI = std::numbers::pi_v<float> / 2.0f;

	float desired_distance;

	float prev_distance_error;
	float prev_angle_error;
	std::chrono::steady_clock::time_point last_follow_time;
	bool was_following = false;  // skip the PID D-term on the first cycle after (re)starting to follow a target

	bool print_extra_info = true;
	bool simulated = configLoader.get<bool>("Simulated");
	std::string robot_DEF = "shadow";

	static constexpr int ODOMETRY_WINDOW_SIZE = 5;
	static constexpr float LINEAR_VELOCITY_DEADBAND  = 2.f;    // mm/s
	static constexpr float ANGULAR_VELOCITY_DEADBAND = 0.01f;  // rad/s
	std::deque<std::tuple<float, float, float>> velocity_window;
	long long last_timestamp = 0;

	std::unique_ptr<DSR::RT_API> rt;

	// ---- Photo-orbit session (orbit_target()) ----
	enum class OrbitStage { TRAVEL, SHOOT_CON, TURN_AWAY, SHOOT_SIN, NEXT_WAYPOINT, DONE };
	OrbitStage orbit_stage = OrbitStage::TRAVEL;
	int orbit_waypoint_idx = 0;
	int orbit_shot_idx = 0;
	bool orbit_bearing_captured = false;
	float orbit_start_bearing = 0.f;  // world-ish bearing (rad) from bump to robot, captured on orbit start
	int photo_counter = 0;            // unique-filename counter, whole session
	bool travel_is_rotating = true;   // TRAVEL's rotate/drive hysteresis latch -- see orbit_target()

	static constexpr float ORBIT_ARRIVAL_TOLERANCE = 0.10f;   // meters
	static constexpr float ORBIT_HEADING_TOLERANCE = 0.035f;  // radians (~2 deg): stop rotating, start driving
	static constexpr float TRAVEL_ROTATE_REENTRY_TOLERANCE = 4.f * ORBIT_HEADING_TOLERANCE;  // ~8 deg: stop driving, start rotating again

	float photo_distance;             // meters, from Photo_distance (mm), set in initialize() like desired_distance
	int orbit_points_k         = configLoader.get<int>("Orbit_points");
	int photos_per_point       = configLoader.get<int>("Photos_per_point");
	float photo_angular_step   = configLoader.get<double>("Photo_angular_step") * std::numbers::pi_v<float> / 180.f;
	std::string photo_save_dir = configLoader.get<std::string>("Photo_save_dir");
	std::string photo_session_dir;  // photo_save_dir/<session_ms>, same id as the orbit log file

	// One CSV row per orbit_target() call (see logs/orbit_<session>.csv) while this feature is
	// being debugged; drop once orbit_target() is validated.
	std::string orbit_log_path;
	void log_orbit(const std::string& csv_line);
	static const char* orbit_stage_name(OrbitStage s);
	// Joins exactly 19 fields (matching the CSV header written in initialize()) with commas,
	// so no call site has to hand-count empty placeholders between the fields it does fill in.
	static std::string csv_row(const std::array<std::string, 19>& fields);

signals:
	//void customSignal();
};

#endif
