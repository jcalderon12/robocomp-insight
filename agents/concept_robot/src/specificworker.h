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
#include <array>
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <QLabel>
#include <QImage>
#include <QPainter>

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
	 * \param brake_gain: If positive, caps the forward speed at brake_gain * (d - desired) / d
	 * (m/s), the progressive braking of the former PD. 0 disables it.
	 */
	void follow_target(float max_forward_speed_factor = 0.6f, float max_angular_speed_factor = 0.6f,
	                   float desired_distance = 0.5f, float distance_tolerance = -1.f, float brake_gain = 0.f);

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
	 * \brief True when the TARGET concept has a "take_photos" affordance (aff_mission), i.e. this is the photo
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
	std::pair<float, float> wall_detour_subgoal(float gx, float gy);   // rodeo de muros del lidar por su extremo
	// Radio de choque del robot (dwa.pdf: 365 mm de Shadow mas margen). Lo usan el DWA y el freno.
	static constexpr float NAV_ROBOT_RADIUS = 0.40f;           // metros
	static constexpr float NAV_DEFAULT_OBSTACLE_SIZE = 0.25f;  // metros, si el nodo no trae concept_size
	static constexpr float NAV_SIM_TIME = 2.0f;                // segundos que se simula cada arco
	static constexpr float NAV_SIM_STEP = 0.1f;                // segundos por paso de la simulacion
	// Muestreo, pesos y holgura del DWA: valores por defecto, se pueden sobrescribir con Nav_* en el config.
	int   NAV_V_SAMPLES = 11;
	int   NAV_W_SAMPLES = 21;
	static constexpr float NAV_MAX_W = 1.2f;                   // rad/s de giro maximo en los arcos (no en el giro en el sitio)
	float NAV_W_WINDOW = 1.0f;                               // rad/s alrededor del giro actual
	static constexpr float NAV_ACCEL_V = 1.0f;                 // m/s^2; el giro no se limita
	static constexpr float NAV_BRAKE_DECEL = 0.5f;             // m/s^2: fuera el arco que no puede parar antes del obstaculo
	// Costes de dwa.pdf, cada uno en 0..1 salvo la distancia: rumbo final a la meta (/pi), distancia
	// final a la meta (/NAV_DIST_NORM), holgura ((1 - d/NAV_SAFE_CLEARANCE)^2), velocidad (1 - v/max),
	// espacio libre en recta al final del arco (hasta NAV_FREE_AHEAD), pasar un obstaculo por el lado
	// contrario al del ciclo anterior, y cambio de giro.
	float NAV_W_HEADING = 1.0f;
	float NAV_W_DIST = 1.0f;
	float NAV_W_CLEARANCE = 3.0f;
	float NAV_W_SPEED = 1.0f;
	float NAV_W_FREE = 1.5f;
	float NAV_W_SIDE = 1.5f;
	float NAV_W_SMOOTH = 1.0f;
	static constexpr float NAV_DIST_NORM = 2.0f;               // metros
	float NAV_SAFE_CLEARANCE = 0.5f;          // metros a partir de los que la holgura no penaliza
	float NAV_FREE_AHEAD = 1.5f;              // metros
	// Regla de progreso: a mas de NAV_PROGRESS_GOAL_DIST de la meta, arcos por debajo de
	// NAV_PROGRESS_MIN_V solo si no hay uno mas rapido libre.
	static constexpr float NAV_PROGRESS_GOAL_DIST = 0.1f;      // metros
	static constexpr float NAV_PROGRESS_MIN_V = 0.15f;         // m/s
	// Rodeo de muros (wall_detour_subgoal()): tramos con huecos menores que NAV_CLUSTER_GAP (por donde el
	// robot no cabe); cortan la recta si pasan a menos de radio + NAV_WALL_PASS_MARGIN; un extremo a
	// menos de NAV_WALL_RANGE_GUARD del alcance del lidar no cuenta (el muro puede seguir).
	static constexpr float NAV_CLUSTER_GAP = 0.9f;             // metros
	static constexpr float NAV_WALL_PASS_MARGIN = 0.25f;       // metros (radio del rodeo de la punta)
	static constexpr float NAV_WALL_BLOCK_MARGIN = 0.05f;      // metros: corta la recta lo que pasa a menos de radio + esto
	static constexpr float NAV_WALL_RANGE_GUARD = 0.3f;        // metros
	static constexpr float NAV_WALL_RELEASE_MARGIN = 0.2f;     // metros de mas para dar la recta por libre
	static constexpr float NAV_WALL_MIN_LENGTH = 0.5f;         // metros: tramos mas cortos no se rodean
	static constexpr float NAV_DETOUR_MAX_ANGLE = 1.0f;        // radianes (~57 grados) de desvio maximo hacia la submeta
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
	std::chrono::steady_clock::time_point nav_rotate_since;   // traza [giro]: inicio del giro en el sitio
	int  nav_turn_sign = 0;        // sentido de giro bloqueado: 0 libre, +1/-1 el signo del rumbo
	int  nav_detour_side = 0;      // lado del rodeo: 0 ninguno, +1 derecha, -1 izquierda
	int  nav_wall_side = 0;        // lado del rodeo de un muro del lidar, igual convenio
	// Trazas del rodeo: inicio, ciclos y el ultimo punto que cortaba (along, lateral, extremo).
	std::chrono::steady_clock::time_point nav_wall_since;
	int nav_wall_cycles = 0;
	float nav_wall_cut_along = 0.f, nav_wall_cut_lateral = 0.f, nav_wall_end_dist = 0.f;
	// Estado del rodeo (tipo Bug2), en root: meta y esquina al empezar, distancia a la meta al empezar y la
	// mejor desde entonces. Se suelta con la recta libre y mas cerca que al empezar (o NAV_WALL_FREE_SECONDS
	// libre); se rehace si la meta se mueve NAV_WALL_GOAL_MOVED; sin progreso (NAV_WALL_ABANDON_DIST mas lejos
	// que al empezar o NAV_WALL_STALL_SECONDS sin mejorar) prueba el otro lado y, si tampoco, lo deja al DWA.
	static constexpr float NAV_WALL_GOAL_MOVED = 1.0f;         // metros
	static constexpr float NAV_WALL_ABANDON_DIST = 1.5f;       // metros
	static constexpr float NAV_WALL_STALL_SECONDS = 10.f;
	static constexpr float NAV_WALL_PROGRESS_STEP = 0.1f;      // metros de mejora que cuentan como progreso
	static constexpr float NAV_WALL_FREE_SECONDS = 2.f;
	static constexpr float NAV_WALL_LEAVE_FREE_SECONDS = 0.3f; // recta libre minima para soltar por progreso
	static constexpr float NAV_WALL_LEAVE_MARGIN = 0.3f;       // metros mas cerca que al empezar para soltar
	static constexpr long  NAV_WALL_DIR_POINTS = 2;            // puntos hacia dentro para la direccion del muro
	float nav_wall_dir_wx = 0.f, nav_wall_dir_wy = 0.f;        // direccion del muro hacia fuera en la esquina (root)
	float nav_target_x = 0.f, nav_target_y = 0.f;              // objetivo (persona) en el marco del robot
	static constexpr float NAV_WALL_GIVEUP_COOLDOWN = 5.f;     // segundos sin empezar otro rodeo tras rendirse
	float nav_wall_goal_wx = 0.f, nav_wall_goal_wy = 0.f;
	float nav_wall_corner_wx = 0.f, nav_wall_corner_wy = 0.f;
	bool  nav_wall_corner_valid = false;
	float nav_wall_hit_dist = 0.f, nav_wall_best_dist = 0.f;
	std::chrono::steady_clock::time_point nav_wall_best_time, nav_wall_free_since, nav_wall_cooldown_until;
	bool  nav_wall_free = false;
	int   nav_wall_switches = 0;
	bool robot_world_pose(float &x, float &y, float &yaw);   // pose del robot en root (DSR)
	int  nav_pass_side = 0;        // lado por el que el arco elegido deja el obstaculo cercano (+1 derecha)
	bool nav_blocked_warned = false;
	std::chrono::steady_clock::time_point nav_last_time;

	// ---- Obstaculos del lidar (paredes, cajas: lo que no esta en el DSR) ----
	// Montaje de Shadow.proto: los dos lidars van invertidos con la misma rotacion, que lleva el
	// punto del dispositivo (x, y, z) al cuerpo como (y, x, -z); el cuerpo es x derecha, y avance.
	// Solo simulacion: en el robot real el componente del lidar entrega otro marco.
	struct LidarMount { const char *name; float tx, ty, tz; };
	static constexpr LidarMount LIDAR_MOUNTS[] = {{"helios", 0.f, -0.155f, 1.075f}, {"bpearl", 0.f, 0.14f, 0.67f}};
	static constexpr int   LIDAR_BINS = 180;                   // sectores de 2 grados: el punto mas cercano de cada uno
	static constexpr float LIDAR_READ_PERIOD = 0.04f;          // segundos; el lidar publica a 20 Hz
	static constexpr float LIDAR_STALE_SECONDS = 0.5f;         // sin datos nuevos en este tiempo se avisa
	static constexpr float LIDAR_DEBUG_PERIOD = 2.f;           // segundos entre trazas de diagnostico
	static constexpr float BRAKE_SIM_STEP = 0.05f;             // segundos por paso del arco del freno
	void update_lidar_obstacles(const DSR::Node &robot_node);
	bool brake_needed(float v, float w);                       // el arco (v, w) acerca el robot a un punto del lidar
	void apply_safety_brake();
	// Punto del lidar en el marco del robot (metros, ya filtrado); z y el lidar de origen, para las trazas.
	struct LidarPoint { float x, y, z; const char *src; };
	std::vector<LidarPoint> lidar_points;
	// Diagnostico: el punto descartado por bajo (z < Lidar_min_height) mas alto del pasillo de delante.
	static constexpr float LIDAR_LOW_CORRIDOR_HALF = 0.5f;     // metros a cada lado
	static constexpr float LIDAR_LOW_CORRIDOR_LENGTH = 2.f;    // metros por delante
	std::optional<LidarPoint> lidar_low_front;
	// Persistencia: un punto solo cuenta si la lectura anterior del mismo lidar tenia otro en su celda
	// o en una vecina (rejilla de LIDAR_PERSIST_CELL). Quita retornos sueltos de una sola lectura.
	static constexpr float LIDAR_PERSIST_CELL = 0.15f;         // metros
	static constexpr size_t LIDAR_N_MOUNTS = 2;
	std::array<std::unordered_set<int64_t>, LIDAR_N_MOUNTS> lidar_prev_cells;
	std::array<std::vector<LidarPoint>, LIDAR_N_MOUNTS> lidar_mount_points;   // ultimos confirmados de cada lidar
	std::array<std::vector<LidarPoint>, LIDAR_N_MOUNTS> lidar_mount_target_points;   // confirmados junto al objetivo (linea de vista)
	int lidar_dropped = 0;                                     // descartados por persistencia desde la ultima traza
	std::optional<LidarPoint> lidar_dropped_nearest;
	// Aislamiento: en la misma lectura (en planta, xy) tiene que haber otro punto en su celda o en una vecina.
	// Quita rayos sueltos que se repiten en el mismo sitio y la persistencia deja pasar.
	int lidar_isolated = 0;                                    // descartados por aislados desde la ultima traza
	std::optional<LidarPoint> lidar_isolated_nearest;
	// Traza del DWA: avance recortado a menos de esta fraccion del posible, con al menos este posible.
	static constexpr float NAV_CUT_TRACE_FRACTION = 0.5f;
	static constexpr float NAV_CUT_TRACE_MIN_V = 0.2f;         // m/s
	bool nav_cut_active = false;

	// ---- Ventana de depuracion de la navegacion (Nav_debug_view) ----
	// navigate_to guarda en nav_dbg los arcos del ciclo y draw_nav_debug() los pinta, en el marco del
	// robot, a NAV_DEBUG_PERIOD como mucho. Con Nav_debug_save_dir no vacio, ademas guarda cada imagen.
	struct NavDebugArc { std::vector<std::pair<float, float>> pts; int status; };   // 0 choca, 1 no frena, 2 valido
	struct NavDebug
	{
		bool navigating = false, rotating = false;
		std::pair<float, float> goal{0.f, 0.f}, subgoal{0.f, 0.f};
		std::vector<Obstacle> dsr;                 // radio ya inflado con NAV_ROBOT_RADIUS
		std::vector<NavDebugArc> arcs;
		int chosen = -1, n_valid = 0;
		float v_target = 0.f, v_cmd = 0.f, w = 0.f, min_clear = -1.f;
		std::vector<int> point_cluster;            // tramo de cada punto de lidar_points
		std::vector<char> cluster_wall;            // tramo con longitud de muro
		int blocking_cluster = -1;
	};
	static constexpr float NAV_DEBUG_PERIOD = 0.1f;            // segundos entre imagenes
	static constexpr int NAV_DEBUG_W = 700, NAV_DEBUG_H = 820;  // pixeles de la imagen (y tamano minimo de la ventana)
	static constexpr int   NAV_DEBUG_ARC_STRIDE = 2;           // pasos del arco por punto pintado
	void draw_nav_debug();
	std::optional<LidarPoint> wall_between_robot_and_target();
	bool nav_debug_view = false;
	std::string nav_debug_save_dir;
	NavDebug nav_dbg;
	std::optional<std::pair<float, float>> lidar_target;      // objetivo excluido del lidar, marco del robot
	std::unique_ptr<QLabel> nav_debug_label;
	std::chrono::steady_clock::time_point nav_debug_last;
	int nav_debug_frame = 0;
	std::chrono::steady_clock::time_point lidar_last_read, lidar_last_data, lidar_last_debug;
	bool lidar_stale_warned = false;
	bool brake_active = false;
	std::chrono::steady_clock::time_point brake_start;
	bool  lidar_enabled;
	bool  lidar_debug;
	float lidar_max_range;          // metros
	float lidar_min_height;         // metros sobre el suelo: por debajo, suelo y baches
	float lidar_max_height;         // metros
	float lidar_self_radius;        // metros: mas cerca del centro es el propio robot
	float lidar_target_exclusion;   // metros alrededor del objetivo: no se esquiva a quien se sigue
	float brake_time;               // segundos de avance que mira el freno
	float brake_margin;             // metros extra al final del arco del freno

	// ---- Fin de follow_person por permanencia a la distancia deseada ----
	// Holgura sobre desired_distance: la aproximacion es asintotica y los ultimos centimetros se
	// recorren a milimetros por segundo, asi que exigir la distancia exacta puede no cumplirse nunca.
	static constexpr float FOLLOW_REACHED_MARGIN = 0.1f;   // metros
	// Tope de avance en follow_person: 0.825 m/s; a 0.6 m/s la botella no llegaba a caer. A mas velocidad
	// el bache 1 se pisa mas fuerte y el robot recorre mas hasta que aparece 'problem'.
	static constexpr float FOLLOW_SPEED_FACTOR = 0.55f;
	static constexpr float FOLLOW_BRAKE_GAIN = 1.2f;       // m/s, el Kp_lin del PD antiguo
	// Deriva de la distancia durante la espera que se interpreta como que la persona se ha movido.
	static constexpr float FOLLOW_HOLD_DRIFT     = 0.25f;  // metros
	// Linea de vista para dar por alcanzada a la persona (wall_between_robot_and_target()): tramos del lidar
	// con huecos menores que LIDAR_POINT_GAP y al menos FOLLOW_LOS_WALL_LENGTH de largo (muros, no la persona)
	// que cruzan la recta a menos de FOLLOW_LOS_HALF_WIDTH antes de FOLLOW_LOS_TARGET_MARGIN del objetivo, o
	// que pasan a menos de FOLLOW_LOS_TARGET_ON_WALL de el (su RT cayo en el muro).
	static constexpr float LIDAR_POINT_GAP = 0.3f;              // metros
	static constexpr float FOLLOW_LOS_WALL_LENGTH = 0.8f;       // metros
	static constexpr float FOLLOW_LOS_HALF_WIDTH = 0.25f;       // metros
	static constexpr float FOLLOW_LOS_TARGET_MARGIN = 0.1f;     // metros
	static constexpr float FOLLOW_LOS_TARGET_ON_WALL = 0.15f;   // metros
	static constexpr float FOLLOW_LOS_SIDE_SPAN = 0.3f;         // metros a cada lado de la vista para "muro que sigue"
	const char *follow_los_reason = "";                         // que prueba de la linea de vista ha fallado (traza)
	bool follow_los_warned = false;
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
	// Traza [persona] (simulacion): pose real de la persona y del robot en Webots frente a la RT publicada.
	std::string person_DEF = "HUMAN_2";
	static constexpr float PERSON_TRACE_PERIOD = 0.5f;          // segundos
	std::chrono::steady_clock::time_point person_trace_last;
	bool person_trace_warned = false;
	void trace_person_ground_truth(const DSR::Node &robot_node);

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
	bool force_photos = false;   // Force_photos: hacer las fotos aunque la verificacion sea negativa
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
