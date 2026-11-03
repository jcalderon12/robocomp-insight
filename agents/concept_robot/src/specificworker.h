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
#include <limits>
#include <utility>

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
	 * \brief Takes the robot to desired_distance of the TARGET and leaves it facing it. Away from
	 * that distance it drives with navigate_to() to the point of the robot-target line at
	 * desired_distance, avoiding the known obstacles; at it, it only turns in place. Never reverses.
	 * \param max_forward_speed_factor: Maximum forward speed factor to apply to the robot (between 0 and 1).
	 * \param max_angular_speed_factor: Maximum angular speed factor to apply to the robot (between 0 and 1).
	 * \param desired_distance: Desired distance to the target in meters.
	 * \param distance_tolerance: Band around desired_distance that counts as reached. Negative:
	 * any distance up to desired_distance counts (follow_person).
	 */
	void follow_target(float max_forward_speed_factor = 0.6f, float max_angular_speed_factor = 0.6f,
	                   float desired_distance = 0.5f, float distance_tolerance = -1.f);

	/**
	 * \brief Dynamic Window Approach: samples (forward, turn) pairs reachable from the current
	 * speed, simulates each arc NAV_SIM_TIME seconds, discards the ones that hit a known obstacle
	 * and returns the best by distance and heading to the goal, clearance and speed. Goal in the
	 * robot frame (meters, forward = second coordinate). Turns in place first if the goal is aside.
	 */
	std::pair<float, float> navigate_to(const DSR::Node &robot_node, float gx, float gy, float max_v, float max_w);

	/**
	 * \brief In-place turn speed towards a bearing, with the turning side locked near +-180 deg.
	 */
	float turn_towards(float angle, float max_w);

	/**
	 * \brief True unless the TARGET node says perceived=false (its RT comes from memory).
	 */
	bool target_perceived();

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
	 * \brief Distance from the robot to the current TARGET, or -1 if it cannot be read.
	 */
	float target_distance();

	/**
	 * \brief Node the current TARGET points at, i.e. the concept this mission is about.
	 */
	std::optional<DSR::Node> get_target_concept_node();

	/**
	 * \brief Asks the concept's agent to verify that the concept is really there, by setting
	 * verify_request on its node.
	 */
	void request_verification();

	/**
	 * \brief Reads verify_result from the concept node: "ok", "fail", or empty if not answered.
	 */
	std::string read_verify_result();

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
	bool imu_nan_warned = false;  // el aviso de lectura no finita se da una sola vez

	std::vector<float> last_velocities_readed;
	std::vector<float> last_robot_pose;

	std::vector<float> last_odometry;

	static constexpr float HALF_PI = std::numbers::pi_v<float> / 2.0f;

	float desired_distance;

	// ---- Navegacion (navigate_to(), DWA) ----
	struct Obstacle { float x, y, radius; };   // marco del robot, metros; radio ya inflado
	std::vector<Obstacle> collect_obstacles(const DSR::Node &robot_node);
	std::pair<float, float> detour_subgoal(float gx, float gy, const std::vector<Obstacle> &obstacles);
	// Huella del robot como circulo: ruedas en +-0.26 m de lado y -0.10/+0.16 m a lo largo.
	static constexpr float NAV_ROBOT_RADIUS = 0.35f;           // metros
	static constexpr float NAV_SAFETY_MARGIN = 0.10f;          // metros de holgura extra al obstaculo
	static constexpr float NAV_DEFAULT_OBSTACLE_SIZE = 0.25f;  // metros, si el nodo no trae concept_size
	static constexpr float NAV_SIM_TIME = 2.0f;                // segundos que se simula cada arco
	static constexpr float NAV_SIM_STEP = 0.1f;                // segundos por paso de la simulacion
	static constexpr int   NAV_V_SAMPLES = 6;
	static constexpr int   NAV_W_SAMPLES = 21;
	static constexpr float NAV_ACCEL_V = 1.0f;                 // m/s^2; el giro no se limita
	// Pesos del coste: distancia final a la meta (m), error de rumbo (rad), cercania a obstaculos
	// (0..1 por debajo de NAV_CLEARANCE_CAP), velocidad (0..1) y cambio de giro (rad/s).
	static constexpr float NAV_W_GOAL = 1.0f;
	static constexpr float NAV_W_HEADING = 0.3f;
	static constexpr float NAV_W_CLEARANCE = 0.3f;
	static constexpr float NAV_W_SPEED = 0.2f;
	static constexpr float NAV_W_SMOOTH = 0.05f;
	static constexpr float NAV_CLEARANCE_CAP = 0.3f;           // metros a partir de los que no penaliza
	// Rodeo (detour_subgoal()): holgura sobre el radio inflado y prolongacion de la tangente.
	static constexpr float NAV_DETOUR_MARGIN = 0.25f;          // metros
	static constexpr float NAV_DETOUR_EXTENSION = 0.5f;        // metros
	// Meta mas de lado que esto: girar en el sitio antes de avanzar, hasta tenerla dentro de DONE.
	static constexpr float NAV_ROTATE_FIRST_ANGLE = 1.3f;      // radianes (~75 grados)
	static constexpr float NAV_ROTATE_DONE_ANGLE = 0.5f;       // radianes (~30 grados)
	// Bloqueo del sentido de giro cerca del antipodal (turn_towards()).
	static constexpr float NAV_TURN_LOCK_ANGLE = 2.6f;         // radianes (~150 grados)
	static constexpr float NAV_TURN_UNLOCK_ANGLE = 1.57f;      // radianes (~90 grados)
	static constexpr float NAV_TURN_GAIN = 1.0f;               // rad/s por radian de error
	bool nav_rotating = false;     // girando en el sitio antes de avanzar
	int  nav_turn_sign = 0;        // sentido de giro bloqueado: 0 libre, +1/-1 el signo del rumbo
	int  nav_detour_side = 0;      // lado del rodeo: 0 ninguno, +1 derecha, -1 izquierda
	bool nav_blocked_warned = false;
	std::chrono::steady_clock::time_point nav_last_time;

	// ---- Fin de follow_person por permanencia a la distancia deseada ----
	// Holgura sobre desired_distance: la aproximacion es asintotica y los ultimos centimetros se
	// recorren a milimetros por segundo, asi que exigir la distancia exacta puede no cumplirse nunca.
	static constexpr float FOLLOW_REACHED_MARGIN = 0.1f;   // metros
	// Deriva de la distancia durante la espera que se interpreta como que la persona se ha movido.
	static constexpr float FOLLOW_HOLD_DRIFT     = 0.25f;  // metros
	float follow_hold_seconds;           // segundos que hay que aguantar a la distancia deseada
	// Un vector nulo no es una posicion, es ausencia de dato: el puente de Webots devuelve la pose
	// a ceros cuando no resuelve el DEF, y asi el objetivo parece estar encima y centrado.
	static constexpr float TARGET_NULL_DISTANCE = 1e-3f;  // metros
	// Edad maxima de la medida para darla por valida al decidir que la mision esta cumplida.
	static constexpr float TARGET_MAX_AGE = 0.5f;         // segundos
	float last_target_distance = -1.f;   // distancia medida en el ultimo follow_target()
	float last_target_angle = 0.f;       // rumbo al objetivo medido en el ultimo follow_target()
	std::chrono::steady_clock::time_point last_target_stamp;  // cuando se midio
	bool  target_null_warned = false;    // el aviso de objetivo nulo se da una sola vez
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
	enum class SpinStage { APPROACH, VERIFY, TURNING, SETTLING, DONE, ABORTED };
	SpinStage spin_stage = SpinStage::APPROACH;
	float spin_accumulated = 0.f;       // radianes girados en total en esta vuelta
	float spin_since_shot = 0.f;        // radianes girados desde la última parada de disparo
	float spin_last_heading = 0.f;      // rumbo del ciclo anterior, para acumular el giro
	bool spin_heading_valid = false;
	std::chrono::steady_clock::time_point spin_settle_start;

	// Aproximacion previa al giro: follow_target coloca al robot a la distancia pedida y mirando
	// al concepto, que es lo que la verificacion necesita. El factor angular va limitado: a
	// velocidad maxima el robot gira 23 grados entre dos lecturas de la RT y se pasa del rumbo.
	static constexpr float PHOTO_APPROACH_MARGIN = 0.2f;            // metros de holgura en la distancia
	static constexpr float PHOTO_APPROACH_ANGLE_TOLERANCE = 0.26f;  // radianes (~15 grados) para darlo por encarado
	static constexpr float PHOTO_APPROACH_SPEED_FACTOR = 0.3f;      // factor lineal
	static constexpr float PHOTO_APPROACH_ANGULAR_FACTOR = 0.15f;   // factor angular -> 3.5 grados por lectura
	float photo_approach_distance; // metros a los que se coloca antes de girar
	// Tope de tiempo: sin el una aproximacion que no converge deja la mision colgada para siempre.
	static constexpr float PHOTO_APPROACH_TIMEOUT = 60.f;          // segundos
	std::chrono::steady_clock::time_point approach_start;          // para el tope de tiempo
	// Verificacion: se pide al agente del concepto y se espera su veredicto, con tope de tiempo
	// porque la primera vez tiene que cargar el modelo.
	float verify_timeout_seconds;
	std::chrono::steady_clock::time_point verify_wait_start;
	bool  approach_started = false;
	std::chrono::steady_clock::time_point approach_check_start;  // ventana de la traza de diagnostico
	float photo_angular_step;     // radianes entre disparos
	float photo_settle_seconds;   // parada en cada paso: es cuando dispara el agente del concepto
	float photo_spin_speed;       // rad/s del giro

signals:
	//void customSignal();
};

#endif
