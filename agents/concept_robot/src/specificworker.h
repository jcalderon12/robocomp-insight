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
	 * \brief True when the current TARGET points at the "bump" node, i.e. this is the photo
	 * mission (photo_spin()) and not a plain follow (follow_target()).
	 */
	bool is_photo_target();

	/**
	 * \brief Photo mission: approaches the bump with follow_target(), then spins in place a full
	 * turn stopping every Photo_angular_step degrees to take a picture. Never translates while
	 * spinning. Each shot is labelled from the live angle to the bump: facing it -> "con_bache",
	 * facing away -> "sin_bache", sideways -> discarded.
	 */
	void photo_spin();

	/**
	 * \brief Spins in place at a constant angular speed (linear speed 0, no closed loop).
	 */
	void spin_in_place(float angular_speed);

	/**
	 * \brief Drives straight ahead at a constant speed (angular speed 0, no closed loop).
	 */
	void drive_forward(float speed);

	/**
	 * \brief Grabs one CameraRGBDSimple frame and saves it as .jpg under Photo_save_dir/<session>/<label>/.
	 */
	void take_photo(const std::string& label, float angle_to_bump, float distance_to_bump);

	/**
	 * \brief Publishes photo_session_dir on the concept node and closes the mission.
	 */
	void finish_photo_mission();

	/**
	 * \brief Clears aff_interacting on the affordance reached via TARGET->has_intention, which is
	 * what mission_controller watches to complete the mission. Shared by both missions.
	 */
	void clear_target_affordance();

	/**
	 * \brief Completes follow_person once the robot holds the desired distance to the target for
	 * Follow_hold_seconds; leaving the distance or drifting resets the wait.
	 */
	void check_follow_reached();

	/**
	 * \brief Resets photo_spin() progress so the next photo mission starts from scratch.
	 */
	void reset_photo_spin();

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

	// ---- Fin de follow_person por permanencia a la distancia deseada ----
	// Holgura sobre desired_distance: la aproximacion es asintotica y los ultimos centimetros se
	// recorren a milimetros por segundo, asi que exigir la distancia exacta puede no cumplirse nunca.
	static constexpr float FOLLOW_REACHED_MARGIN = 0.1f;   // metros
	// Deriva de la distancia durante la espera que se interpreta como que la persona se ha movido.
	static constexpr float FOLLOW_HOLD_DRIFT     = 0.25f;  // metros
	float follow_hold_seconds;           // segundos que hay que aguantar a la distancia deseada
	float last_target_distance = -1.f;   // distancia medida en el ultimo follow_target()
	bool  follow_holding = false;        // ya a la distancia deseada, contando
	float follow_hold_distance = 0.f;    // distancia al empezar la espera, para medir la deriva
	std::chrono::steady_clock::time_point follow_hold_start;

	bool print_extra_info = configLoader.get<bool>("print_extra_info");
	bool simulated = configLoader.get<bool>("Simulated");
	std::string robot_DEF = "shadow";

	static constexpr int ODOMETRY_WINDOW_SIZE = 5;
	static constexpr float LINEAR_VELOCITY_DEADBAND  = 2.f;    // mm/s
	static constexpr float ANGULAR_VELOCITY_DEADBAND = 0.01f;  // rad/s
	std::deque<std::tuple<float, float, float>> velocity_window;
	long long last_timestamp = 0;

	std::unique_ptr<DSR::RT_API> rt;

	// ---- Misión de fotos (photo_spin()) ----
	enum class SpinStage { TURNING, SETTLING, DONE };
	SpinStage spin_stage = SpinStage::TURNING;
	float spin_accumulated = 0.f;       // radianes girados en total en esta vuelta
	float spin_since_shot = 0.f;        // radianes girados desde la última parada de disparo
	float spin_last_heading = 0.f;      // rumbo del ciclo anterior, para acumular el giro
	bool spin_heading_valid = false;
	std::chrono::steady_clock::time_point spin_settle_start;
	int photo_counter = 0;

	float photo_angular_step;     // radianes entre disparos
	float photo_front_window;     // radianes; |ángulo al bache| <= esto -> con_bache
	float photo_back_window;      // radianes; |ángulo al bache| >= PI - esto -> sin_bache
	float photo_settle_seconds;   // espera tras parar, antes de leer el ángulo y disparar
	float photo_spin_speed;       // rad/s del giro
	std::string photo_save_dir  = configLoader.get<std::string>("Photo_save_dir");
	std::string photo_session_dir;  // photo_save_dir/<session_ms>
	std::string photo_log_path;     // una fila por disparo

signals:
	//void customSignal();
};

#endif
