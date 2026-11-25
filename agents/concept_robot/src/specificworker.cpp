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
#include <algorithm>
#include <sstream>
#include <filesystem>
#include <fstream>

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	setlocale(LC_NUMERIC, "C");
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

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

	rt = G->get_rt_api();

	last_velocities_readed = getVelocitiesFromDSR();

	nav_last_time = std::chrono::steady_clock::now();

	last_odometry = {0.0f, 0.0f, 0.0f};

	if (simulated)
		{
			desired_distance = configLoader.get<double>("Desired_distance") / 1000;
			std::cout << "Desired distance (simulated): " << desired_distance << std::endl;
		}
	else
		{
			desired_distance = configLoader.get<double>("Desired_distance");
			std::cout << "Desired distance (real): " << desired_distance << std::endl;
		}

	// Segundos que hay que aguantar a la distancia deseada para dar follow_person por terminada.
	follow_hold_seconds = configLoader.get<double>("Follow_hold_seconds");

	// Parámetros de la misión de fotos: en el config van en grados, aquí se pasan a radianes.
	const float deg2rad = std::numbers::pi_v<float> / 180.f;
	photo_approach_distance = configLoader.get<double>("Photo_approach_distance") / 1000.f;
	verify_timeout_seconds = configLoader.get<double>("Verify_timeout_seconds");
	force_photos = configLoader.get<bool>("Force_photos");
	photo_angular_step  = configLoader.get<double>("Photo_angular_step") * deg2rad;
	photo_settle_seconds = configLoader.get<double>("Photo_settle_seconds");
	photo_spin_speed    = configLoader.get<double>("Photo_spin_speed_factor") * WEBOTS_MAX_ANGULAR_SPEED;

	// Lidar: obstaculos para el DWA y freno de seguridad.
	lidar_enabled          = configLoader.get<bool>("Lidar_enabled");
	lidar_debug            = configLoader.get<bool>("Lidar_debug");
	lidar_max_range        = configLoader.get<double>("Lidar_max_range");
	lidar_min_height       = configLoader.get<double>("Lidar_min_height");
	lidar_max_height       = configLoader.get<double>("Lidar_max_height");
	lidar_self_radius      = configLoader.get<double>("Lidar_self_radius");
	lidar_target_exclusion = configLoader.get<double>("Lidar_target_exclusion");
	brake_time             = configLoader.get<double>("Brake_time");
	brake_margin           = configLoader.get<double>("Brake_margin");
	lidar_last_read = lidar_last_data = lidar_last_debug = std::chrono::steady_clock::now();

	// DWA: muestreo, pesos y holgura; si falta la clave se queda el valor por defecto del .h.
	auto nav_param = [&](const std::string &key, auto &var) {
		if (configLoader.exists(key))
			var = static_cast<std::remove_reference_t<decltype(var)>>(configLoader.get<double>(key));
	};
	nav_param("Nav_v_samples", NAV_V_SAMPLES);
	nav_param("Nav_w_samples", NAV_W_SAMPLES);
	nav_param("Nav_w_window", NAV_W_WINDOW);
	nav_param("Nav_w_heading", NAV_W_HEADING);
	nav_param("Nav_w_dist", NAV_W_DIST);
	nav_param("Nav_w_clearance", NAV_W_CLEARANCE);
	nav_param("Nav_w_speed", NAV_W_SPEED);
	nav_param("Nav_w_free", NAV_W_FREE);
	nav_param("Nav_w_side", NAV_W_SIDE);
	nav_param("Nav_w_smooth", NAV_W_SMOOTH);
	nav_param("Nav_safe_clearance", NAV_SAFE_CLEARANCE);
	nav_param("Nav_free_ahead", NAV_FREE_AHEAD);
	NAV_V_SAMPLES = std::max(NAV_V_SAMPLES, 2);
	NAV_W_SAMPLES = std::max(NAV_W_SAMPLES, 2);

	// Ventana de depuracion de la navegacion (opcional en el config).
	nav_debug_view = configLoader.exists("Nav_debug_view") && configLoader.get<bool>("Nav_debug_view");
	if (configLoader.exists("Nav_debug_save_dir"))
		nav_debug_save_dir = configLoader.get<std::string>("Nav_debug_save_dir");
	if (nav_debug_view)
	{
		nav_debug_label = std::make_unique<QLabel>();
		nav_debug_label->setWindowTitle("concept_robot: DWA");
		nav_debug_label->setMinimumSize(NAV_DEBUG_W, NAV_DEBUG_H);
		nav_debug_label->show();
		if (!nav_debug_save_dir.empty())
			std::filesystem::create_directories(nav_debug_save_dir);
	}
	nav_debug_last = std::chrono::steady_clock::now();

	std::cout << "Numeric locale active: " << setlocale(LC_NUMERIC, nullptr) << std::endl;
}



void SpecificWorker::compute()
{
	auto_localization();
	if (auto robot_node_opt = G->get_node("robot"); robot_node_opt.has_value())
		trace_person_ground_truth(robot_node_opt.value());

    if (check_affordance_active())
	{
		if (auto robot_node_opt = G->get_node("robot"); robot_node_opt.has_value())
			update_lidar_obstacles(robot_node_opt.value());
		// Si el TARGET es el bache, la misión es la de fotos; si no, seguimiento normal.
		if (is_photo_target())
			photo_spin();
		else
		{
			reset_photo_spin();
			follow_target(FOLLOW_SPEED_FACTOR, 1.0f, desired_distance, -1.f, FOLLOW_BRAKE_GAIN);
			check_follow_reached();
		}
	}
	else{
		if (nav_debug_view)
			if (auto robot_node_opt = G->get_node("robot"); robot_node_opt.has_value())
				update_lidar_obstacles(robot_node_opt.value());
		stop_robot();
		nav_rotating = false;
		nav_turn_sign = 0;
		nav_detour_side = 0;
		nav_wall_side = 0;
		nav_pass_side = 0;
		follow_holding = false;
		last_target_distance = -1.f;
		reset_photo_spin();
		brake_active = false;
	}

	// Ultima barrera antes de mandar la velocidad: solo actua si el avance pedido choca.
	apply_safety_brake();
	if (nav_debug_view)
		draw_nav_debug();

	std::vector<float> actual_velocities = getVelocitiesFromDSR();

	if (has_significant_change(actual_velocities, last_velocities_readed)) {
		this->omnirobot_proxy->setSpeedBase(0.0, actual_velocities[0], actual_velocities[1]);
		if(print_extra_info)
			std::cout << "setSpeedBase -> advx: " << actual_velocities[0] << " | rot: " << actual_velocities[1] << std::endl;
	}

	last_velocities_readed = actual_velocities;	
	

	update_or_create_imu_node();

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

#pragma region ROBOT_METHODS

void SpecificWorker::follow_target(float max_forward_speed_factor, float max_angular_speed_factor, float desired_distance, float distance_tolerance, float brake_gain)
{
    auto robot_node_opt = G->get_node("robot");
    if (!robot_node_opt.has_value())
    {
        std::cerr << "Robot node not found in DSR." << std::endl;
        return;
    }
    DSR::Node robot_node = robot_node_opt.value();

    auto target_edges = G->get_edges_by_type("TARGET");
    if (target_edges.empty())
    {
        std::cerr << "No target edges found in DSR." << std::endl;
        return;
    }
    DSR::Edge target_edge = target_edges[0];

    // rt_translation es un buffer circular de 5 muestras (15 floats) y rt_head_index dice cual es
    // la nueva: leerlo a pelo devolvia siempre el hueco 0, que solo se reescribe una de cada cinco
    // publicaciones. get_translation con Nearest resuelve el buffer y da la muestra actual.
    auto t_opt = rt->get_translation(robot_node, target_edge.to());
    if (!t_opt.has_value())
    {
        std::cerr << "RT translation missing." << std::endl;
        return;
    }

    float x = static_cast<float>(t_opt.value().x());
    float y = static_cast<float>(t_opt.value().y());

    float distance_to_target = std::sqrt(x*x + y*y);
    float angle_to_target    = std::atan2(x, y);

    // Objetivo en el origen: el publicador perdio la pose y manda ceros. Tratarlo como medida
    // valida daria error de distancia nulo y atan2(0,0)=0, o sea "alcanzado y centrado".
    if (distance_to_target < TARGET_NULL_DISTANCE)
    {
        if (!target_null_warned)
        {
            std::cerr << "follow_target: RT del objetivo a cero; se ignora la medida." << std::endl;
            target_null_warned = true;
        }
        stop_robot();
        last_target_distance = -1.f;
        return;
    }
    target_null_warned = false;

    last_target_distance = distance_to_target;   // la lee check_follow_reached()
    last_target_angle    = angle_to_target;      // la lee la etapa APPROACH
    last_target_stamp    = std::chrono::steady_clock::now();

    const float max_v = WEBOTS_MAX_LINEAR_SPEED * max_forward_speed_factor;
    const float max_w = WEBOTS_MAX_ANGULAR_SPEED * max_angular_speed_factor;

    // Ya a distancia: solo queda encararlo, girando en el sitio. Con tolerancia negativa vale
    // cualquier distancia menor que la deseada, porque el robot nunca va marcha atras.
    const bool at_distance = distance_tolerance < 0.f
        ? distance_to_target <= desired_distance
        : std::abs(distance_to_target - desired_distance) <= distance_tolerance;

    float linear_velocity = 0.f, angular_velocity = 0.f;
    if (at_distance)
    {
        nav_rotating = false;
        angular_velocity = turn_towards(angle_to_target, max_w);
    }
    else
    {
        // Meta: el punto de la recta robot-objetivo a desired_distance del objetivo. Si el robot
        // esta mas cerca, la meta cae a su espalda y la alcanza dando la vuelta.
        const float scale = 1.f - desired_distance / distance_to_target;
        // Frenado progresivo del PD de antes del DWA: v <= gain * (d - deseada) / d.
        float nav_max_v = max_v;
        if (brake_gain > 0.f && distance_to_target > desired_distance)
            nav_max_v = std::min(max_v, brake_gain * (distance_to_target - desired_distance) / distance_to_target);
        nav_target_x = x;   // el objetivo (la persona), para que el rodeo sepa si se ha movido de verdad
        nav_target_y = y;
        std::tie(linear_velocity, angular_velocity) = navigate_to(robot_node, x * scale, y * scale, nav_max_v, max_w);
    }

    if (print_extra_info)
    {
        auto ts_robot = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        std::cout << "[" << ts_robot << "] RT target translation -> x: " << x << " | y: " << y
                  << " | angle_to_target: " << angle_to_target
                  << " | linear_v: " << linear_velocity
                  << " | angular_v: " << angular_velocity << std::endl;
    }

	G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node, linear_velocity);
	G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node, angular_velocity);
	G->update_node(robot_node);
}

float SpecificWorker::turn_towards(float angle, float max_w)
{
	// Cerca de +-180 grados el rumbo cambia de signo con cualquier ruido y el robot no decidiria
	// hacia donde girar: pasado NAV_TURN_LOCK_ANGLE se fija el sentido hasta tenerlo delante.
	const float PI = std::numbers::pi_v<float>;
	const int previous_sign = nav_turn_sign;
	if (std::abs(angle) < NAV_TURN_UNLOCK_ANGLE)
		nav_turn_sign = 0;
	else if (nav_turn_sign == 0 && std::abs(angle) > NAV_TURN_LOCK_ANGLE)
		nav_turn_sign = angle > 0.f ? 1 : -1;
	if (nav_turn_sign != previous_sign && lidar_debug)
		std::cout << "[giro] sentido " << (nav_turn_sign == 0 ? "libre" : nav_turn_sign > 0 ? "bloqueado a la derecha" : "bloqueado a la izquierda")
		          << " con la meta a " << angle * 180.f / PI << " grados" << std::endl;

	if (nav_turn_sign != 0 && (angle > 0.f ? 1 : -1) != nav_turn_sign)
		angle += nav_turn_sign * 2.f * PI;   // sigue por el mismo lado aunque el rumbo haya saltado

	// Rumbo positivo = objetivo a la derecha = giro negativo.
	return std::clamp(-NAV_TURN_GAIN * angle, -max_w, max_w);
}

std::vector<SpecificWorker::Obstacle> SpecificWorker::collect_obstacles(const DSR::Node &robot_node)
{
	// Obstaculos conocidos: los nodos "object" del DSR con RT desde el robot, en su marco. Un
	// objeto que aun no esta en el DSR no se esquiva, que es lo que hace que el primer bache se pise.
	std::vector<Obstacle> obstacles;
	for (const auto &node : G->get_nodes_by_type("object"))
	{
		auto t_opt = rt->get_translation(robot_node, node.id());
		if (!t_opt.has_value())
			continue;
		const float ox = static_cast<float>(t_opt.value().x());
		const float oy = static_cast<float>(t_opt.value().y());
		if (std::hypot(ox, oy) < TARGET_NULL_DISTANCE)
			continue;   // RT a ceros: ausencia de dato

		// Tamaño medido por la simulacion (metros); el mayor de los dos lados en planta.
		float size = NAV_DEFAULT_OBSTACLE_SIZE;
		auto size_it = node.attrs().find("concept_size");
		if (size_it != node.attrs().end())
		{
			try
			{
				const auto &s = std::get<std::vector<float>>(size_it->second.value());
				if (s.size() >= 2)
					size = std::max(s[0], s[1]);
			}
			catch (const std::bad_variant_access&) {}
		}
		obstacles.push_back({ox, oy, size / 2.f + NAV_ROBOT_RADIUS});   // el margen lo pone el coste de holgura
	}
	return obstacles;
}

std::pair<float, float> SpecificWorker::detour_subgoal(float gx, float gy, const std::vector<Obstacle> &obstacles)
{
	// Sin esto el DWA se queda parado delante de un obstaculo que tapa la meta: acercarse empeora la
	// holgura tanto como mejora la distancia. Se apunta a la tangente del primero que corta la recta.
	const float goal_dist = std::hypot(gx, gy);
	if (goal_dist < 1e-3f)
		return {gx, gy};
	const float ux = gx / goal_dist, uy = gy / goal_dist;

	const Obstacle *blocking = nullptr;
	float blocking_along = goal_dist;
	for (const auto &o : obstacles)
	{
		const float along = o.x * ux + o.y * uy;
		if (along <= 0.f || along >= blocking_along)
			continue;
		if (std::abs(o.x * uy - o.y * ux) >= o.radius)
			continue;   // la recta pasa por fuera
		if (std::hypot(gx - o.x, gy - o.y) < o.radius || std::hypot(o.x, o.y) < o.radius)
			continue;   // meta o robot dentro del margen: rodear no lo arregla
		blocking = &o;
		blocking_along = along;
	}
	if (blocking == nullptr)
	{
		if (nav_detour_side != 0 && lidar_debug)
			std::cout << "[desvio DSR] recta a la meta libre; fin del desvio" << std::endl;
		nav_detour_side = 0;
		return {gx, gy};
	}

	// Tangente desde el robot al circulo de radio inflado + NAV_DETOUR_MARGIN, prolongada para que
	// el robot la rebase. Rumbos medidos como el resto: atan2(x, y), positivo a la derecha.
	const float D = std::hypot(blocking->x, blocking->y);
	const float r = blocking->radius + NAV_DETOUR_MARGIN;
	const float base = std::atan2(blocking->x, blocking->y);
	const float alpha = std::asin(std::min(1.f, r / D));
	const float length = std::sqrt(std::max(D * D - r * r, 0.f)) + NAV_DETOUR_EXTENSION;
	auto candidate = [&](int side) {
		const float b = base + side * alpha;
		return std::pair<float, float>{length * std::sin(b), length * std::cos(b)};
	};

	// El lado se elige una vez, el del camino mas corto, y se mantiene mientras siga tapando.
	if (nav_detour_side == 0)
	{
		auto path_length = [&](int side) {
			auto [cx, cy] = candidate(side);
			return std::hypot(cx, cy) + std::hypot(gx - cx, gy - cy);
		};
		nav_detour_side = path_length(1) <= path_length(-1) ? 1 : -1;
		if (lidar_debug)
		{
			const auto sub = candidate(nav_detour_side);
			std::cout << "[desvio DSR] nodo en (" << blocking->x << ", " << blocking->y << ") radio " << blocking->radius
			          << " corta la recta a " << blocking_along << " m; lado " << (nav_detour_side > 0 ? "derecho" : "izquierdo")
			          << " -> submeta (" << sub.first << ", " << sub.second << "); meta a " << goal_dist << " m" << std::endl;
		}
	}
	return candidate(nav_detour_side);
}

bool SpecificWorker::robot_world_pose(float &x, float &y, float &yaw)
{
	// Pose del robot en root (DSR). Ejes como en concept_person: avance (cos yaw, -sin yaw), derecha
	// (sin yaw, cos yaw); auto_localization() intercambia x/y de la traslacion y deja el cuaternion.
	auto robot_opt = G->get_node("robot");
	auto root_opt = G->get_node("root");
	if (!robot_opt.has_value() || !root_opt.has_value())
		return false;
	auto t_opt = rt->get_translation(root_opt.value(), robot_opt.value().id());
	auto edge_opt = rt->get_edge_RT(root_opt.value(), robot_opt.value().id());
	if (!t_opt.has_value() || !edge_opt.has_value())
		return false;
	auto q_opt = G->get_attrib_by_name<rt_quaternion_att>(edge_opt.value());
	if (!q_opt.has_value() || q_opt.value().get().size() < 4)
		return false;
	const auto &q = q_opt.value().get();   // {qx, qy, qz, qw}
	x = static_cast<float>(t_opt.value().x());
	y = static_cast<float>(t_opt.value().y());
	yaw = std::atan2(2.f * (q[3] * q[2] + q[0] * q[1]), 1.f - 2.f * (q[1] * q[1] + q[2] * q[2]));
	return true;
}

std::pair<float, float> SpecificWorker::wall_detour_subgoal(float gx, float gy)
{
	// El DWA solo ve 2 s de arco: un muro entre el robot y la meta lo deja parado delante. Se agrupan
	// los puntos del lidar en tramos y, si uno corta la recta a la meta, se rodea su extremo. El rodeo
	// tiene estado (tipo Bug2): lado, esquina y meta en el mundo y distancia a la meta al empezar.
	// Se suelta por progreso (recta libre y mas cerca que al empezar), no por la geometria de un ciclo;
	// la esquina se reajusta a lo que se ve y el plan se rehace si la meta se mueve o no hay progreso.
	const float goal_dist = std::hypot(gx, gy);
	const size_t N = lidar_points.size();
	const auto now = std::chrono::steady_clock::now();
	const float PI = std::numbers::pi_v<float>;

	float rx = 0.f, ry = 0.f, ryaw = 0.f;
	const bool have_pose = robot_world_pose(rx, ry, ryaw);
	const float pc = std::cos(ryaw), ps = std::sin(ryaw);
	auto to_world = [&](float bx, float by) { return std::pair<float, float>{rx + by * pc + bx * ps, ry - by * ps + bx * pc}; };
	auto to_body = [&](float wx, float wy) {
		const float dx = wx - rx, dy = wy - ry;
		return std::pair<float, float>{dx * ps + dy * pc, dx * pc - dy * ps};
	};

	auto release = [&](const std::string &reason) {
		if (nav_wall_side != 0 && lidar_debug)
			std::cout << "[rodeo] fin del rodeo tras " << nav_wall_cycles << " ciclos ("
			          << std::chrono::duration<float>(now - nav_wall_since).count() << " s); meta a " << goal_dist
			          << " m (al empezar " << nav_wall_hit_dist << "); motivo: " << reason << std::endl;
		nav_wall_side = 0;
		nav_wall_corner_valid = false;
		return std::pair<float, float>{gx, gy};
	};
	if (goal_dist < 1e-3f || N < 2)
		return release("sin puntos del lidar");

	// Tramos: puntos consecutivos en angulo (lidar_points va por sector) a menos de NAV_CLUSTER_GAP,
	// hueco por el que el robot no cabe. El ultimo puede continuar en el primero (por detras).
	auto gap = [&](size_t a, size_t b) {
		return std::hypot(lidar_points[a].x - lidar_points[b].x, lidar_points[a].y - lidar_points[b].y);
	};
	std::vector<std::vector<size_t>> clusters{{0}};
	for (size_t i = 1; i < N; ++i)
	{
		if (gap(i - 1, i) >= NAV_CLUSTER_GAP)
			clusters.emplace_back();
		clusters.back().push_back(i);
	}
	if (clusters.size() > 1 && gap(N - 1, 0) < NAV_CLUSTER_GAP)
	{
		auto &last = clusters.back();
		last.insert(last.end(), clusters.front().begin(), clusters.front().end());
		clusters.front() = std::move(last);
		clusters.pop_back();
	}
	if (nav_debug_view)
	{
		nav_dbg.point_cluster.assign(N, -1);
		nav_dbg.cluster_wall.assign(clusters.size(), 0);
		for (size_t c = 0; c < clusters.size(); ++c)
			for (const size_t i : clusters[c])
				nav_dbg.point_cluster[i] = static_cast<int>(c);
	}

	// El tramo que corta antes la recta, con el pasillo del ancho del robot mas margen.
	const float ux = gx / goal_dist, uy = gy / goal_dist;
	// Corta solo lo que pasa a menos de block de la recta: un muro paralelo a la ruta a 0.6 m no tapa (el robot
	// cabe) y antes disparaba rodeos por la punta del muro, el camino largo. El rodeo de la punta usa pass.
	const float pass = NAV_ROBOT_RADIUS + NAV_WALL_PASS_MARGIN;
	const float block = NAV_ROBOT_RADIUS + NAV_WALL_BLOCK_MARGIN;
	const std::vector<size_t> *blocking = nullptr;
	size_t blocking_k = 0;   // posicion, dentro del tramo, del punto que corta
	float blocking_along = goal_dist - NAV_ROBOT_RADIUS;
	auto find_blocking = [&](float pass_test) {
	for (const auto &c : clusters)
	{
		// Solo muros: lo corto (la persona, una caja pequena) lo esquiva el DWA.
		float min_x = std::numeric_limits<float>::max(), max_x = -min_x, min_y = min_x, max_y = -min_x;
		for (const size_t i : c)
		{
			min_x = std::min(min_x, lidar_points[i].x); max_x = std::max(max_x, lidar_points[i].x);
			min_y = std::min(min_y, lidar_points[i].y); max_y = std::max(max_y, lidar_points[i].y);
		}
		if (std::hypot(max_x - min_x, max_y - min_y) < NAV_WALL_MIN_LENGTH)
			continue;
		if (nav_debug_view)
			nav_dbg.cluster_wall[&c - clusters.data()] = 1;
		for (size_t k = 0; k < c.size(); ++k)
		{
			const auto &p = lidar_points[c[k]];
			const float along = p.x * ux + p.y * uy;
			// Por delante del cuerpo del robot: una pared al lado (along ~ 0) la gestiona el DWA.
			if (along <= NAV_ROBOT_RADIUS || along >= blocking_along || std::abs(p.x * uy - p.y * ux) >= pass_test)
				continue;
			blocking = &c;
			blocking_k = k;
			blocking_along = along;
			nav_wall_cut_lateral = p.x * uy - p.y * ux;
		}
	}
	};
	// Con el rodeo en marcha, la recta solo cuenta como libre con NAV_WALL_RELEASE_MARGIN de mas.
	find_blocking(block);
	if (blocking == nullptr && nav_wall_side != 0)
		find_blocking(block + NAV_WALL_RELEASE_MARGIN);
	if (nav_debug_view)
		nav_dbg.blocking_cluster = blocking ? static_cast<int>(blocking - clusters.data()) : -1;

	// Extremo del tramo por un lado (-1 izquierda, +1 derecha), recorriendolo desde el punto que corta y
	// sin pasar de 90 grados respecto a la direccion de la meta: un muro que sigue por detras del robot
	// no tiene extremo util por ese lado (antes daba submetas a la espalda, al otro lado de la persona).
	struct End { size_t index; size_t inner; bool truncated; };   // inner: un punto antes, hacia dentro del muro
	const float goal_angle = std::atan2(gx, gy);
	auto endpoint = [&](int side) {
		const auto &c = *blocking;
		size_t last = blocking_k;
		bool truncated = false;
		for (long j = static_cast<long>(blocking_k); j >= 0 && j < static_cast<long>(c.size()); j += side)
		{
			const auto &p = lidar_points[c[j]];
			const float rel = std::remainder(std::atan2(p.x, p.y) - goal_angle, 2.f * PI);
			if (side * rel > PI / 2)
			{
				truncated = true;
				break;
			}
			last = static_cast<size_t>(j);
		}
		// Punto interior a NAV_WALL_DIR_POINTS del extremo (hacia el que corta): da la direccion del muro
		const long inner = std::clamp(static_cast<long>(last) - side * NAV_WALL_DIR_POINTS, 0L, static_cast<long>(c.size()) - 1);
		return End{c[last], c[inner], truncated};
	};
	const bool ring = blocking != nullptr && blocking->size() > 2 && gap(blocking->back(), blocking->front()) < NAV_CLUSTER_GAP;
	auto usable = [&](const End &e) {
		const auto &p = lidar_points[e.index];
		return !e.truncated && !ring && std::hypot(p.x, p.y) < lidar_max_range - NAV_WALL_RANGE_GUARD;
	};
	const float r = pass + NAV_DETOUR_MARGIN;
	// Seguro: la submeta nunca a mas de 90 grados de la direccion de la meta (seria alejarse de ella).
	auto clamp_to_goal = [&](std::pair<float, float> s) {
		const float rel = std::remainder(std::atan2(s.first, s.second) - goal_angle, 2.f * PI);
		if (std::abs(rel) <= PI / 2)
			return s;
		const float b = goal_angle + (rel > 0.f ? PI / 2 : -PI / 2), length = std::hypot(s.first, s.second);
		return std::pair<float, float>{length * std::sin(b), length * std::cos(b)};
	};
	auto tangent = [&](float ex, float ey, int side) {
		const float D = std::max(std::hypot(ex, ey), 1e-3f);
		// Pegado al extremo (D < r) la tangente sale a 90 grados y el robot giraria en el sitio: se
		// limita el desvio a NAV_DETOUR_MAX_ANGLE para que lo resuelvan los arcos.
		const float b = std::atan2(ex, ey) + side * std::min(std::asin(std::min(1.f, r / D)), NAV_DETOUR_MAX_ANGLE);
		const float length = std::sqrt(std::max(D * D - r * r, 0.f)) + NAV_DETOUR_EXTENSION;
		return clamp_to_goal({length * std::sin(b), length * std::cos(b)});
	};
	// Junto a la punta (D < r) no hay tangente: punto de paso a r de la punta, siguiendo la direccion del
	// muro hacia fuera. No queda a la espalda mientras el robot no haya rebasado la punta.
	auto pass_point = [&](float ex, float ey, float dx, float dy, int side) {
		if (std::hypot(ex, ey) >= r)
			return tangent(ex, ey, side);
		const float n = std::hypot(dx, dy);
		if (n < 1e-3f)
			return tangent(ex, ey, side);
		return clamp_to_goal({ex + r * dx / n, ey + r * dy / n});
	};
	auto store_corner = [&](const End &e) {
		const auto &p = lidar_points[e.index], &q = lidar_points[e.inner];
		std::tie(nav_wall_corner_wx, nav_wall_corner_wy) = to_world(p.x, p.y);
		const auto [qx, qy] = to_world(q.x, q.y);
		nav_wall_dir_wx = nav_wall_corner_wx - qx;
		nav_wall_dir_wy = nav_wall_corner_wy - qy;
		nav_wall_corner_valid = true;
	};
	// Diagnostico de por que la recta ya no corta (para la traza de fin)
	auto free_reason = [&]() {
		std::ostringstream os;
		os << "recta a la meta libre y ";
		if (goal_dist <= nav_wall_hit_dist - NAV_WALL_LEAVE_MARGIN)
			os << "mas cerca que al empezar";
		else
			os << NAV_WALL_FREE_SECONDS << " s libre";
		return os.str();
	};

	// ---- Rodeo en marcha ----
	if (nav_wall_side != 0 && have_pose)
	{
		// El objetivo (la persona) se ha movido de verdad: se rehace el plan desde cero. Se mira el objetivo y
		// no la meta, que es el punto a la distancia deseada y gira alrededor de el al moverse el robot.
		const auto [gwx, gwy] = to_world(nav_target_x, nav_target_y);
		if (const float moved = std::hypot(gwx - nav_wall_goal_wx, gwy - nav_wall_goal_wy); moved > NAV_WALL_GOAL_MOVED)
		{
			std::ostringstream os;
			os << "el objetivo se ha movido " << moved << " m; se vuelve a decidir";
			release(os.str());
		}
		else
		{
			// Progreso hacia la meta
			if (goal_dist < nav_wall_best_dist - NAV_WALL_PROGRESS_STEP)
			{
				nav_wall_best_dist = goal_dist;
				nav_wall_best_time = now;
			}
			const float stalled = std::chrono::duration<float>(now - nav_wall_best_time).count();
			if (goal_dist > nav_wall_hit_dist + NAV_WALL_ABANDON_DIST || stalled > NAV_WALL_STALL_SECONDS)
			{
				if (nav_wall_switches == 0)
				{
					if (lidar_debug)
						std::cout << "[rodeo] sin progreso por el lado " << (nav_wall_side < 0 ? "izquierdo" : "derecho")
						          << " (meta a " << goal_dist << " m, al empezar " << nav_wall_hit_dist << ", " << stalled
						          << " s sin mejorar); se prueba el otro" << std::endl;
					nav_wall_side = -nav_wall_side;
					nav_wall_switches = 1;
					nav_wall_hit_dist = nav_wall_best_dist = goal_dist;
					nav_wall_best_time = now;
					nav_wall_goal_wx = gwx;
					nav_wall_goal_wy = gwy;
					nav_wall_corner_valid = false;
				}
				else
				{
					nav_wall_cooldown_until = now + std::chrono::milliseconds(static_cast<int>(NAV_WALL_GIVEUP_COOLDOWN * 1000));
					return release("sin progreso por ningun lado; se deja al DWA unos segundos");
				}
			}

			// Recta libre: se suelta si lleva NAV_WALL_LEAVE_FREE_SECONDS libre y el robot esta al menos
			// NAV_WALL_LEAVE_MARGIN mas cerca que al empezar (unos cm de ruido ya no bastan), o si lleva
			// NAV_WALL_FREE_SECONDS libre.
			if (blocking == nullptr)
			{
				if (!nav_wall_free)
				{
					nav_wall_free = true;
					nav_wall_free_since = now;
				}
				const float free_for = std::chrono::duration<float>(now - nav_wall_free_since).count();
				if ((free_for >= NAV_WALL_LEAVE_FREE_SECONDS && goal_dist <= nav_wall_hit_dist - NAV_WALL_LEAVE_MARGIN)
				    || free_for > NAV_WALL_FREE_SECONDS)
					return release(free_reason());
			}
			else
				nav_wall_free = false;

			// Esquina: se reajusta al extremo que se ve por el lado del rodeo; si no hay uno util (el muro
			// sigue fuera del alcance o por detras), se mantiene la guardada.
			if (blocking != nullptr)
			{
				const End e = endpoint(nav_wall_side);
				if (usable(e) || !nav_wall_corner_valid)
					store_corner(e);
				nav_wall_cut_along = blocking_along;
			}
			if (!nav_wall_corner_valid)
				return release("sin esquina que rodear");
			const auto [cx, cy] = to_body(nav_wall_corner_wx, nav_wall_corner_wy);
			// Direccion del muro en el marco del robot (vector, sin la traslacion)
			const auto [ox, oy] = to_body(rx, ry);
			const auto [dwx, dwy] = to_body(rx + nav_wall_dir_wx, ry + nav_wall_dir_wy);
			nav_wall_end_dist = std::hypot(cx, cy);
			++nav_wall_cycles;
			return pass_point(cx, cy, dwx - ox, dwy - oy, nav_wall_side);
		}
	}
	else if (nav_wall_side != 0)
		nav_wall_side = 0;   // sin pose no hay estado: se decide en cada ciclo

	// ---- Empezar un rodeo ----
	if (blocking == nullptr || now < nav_wall_cooldown_until)
		return {gx, gy};
	const End left = endpoint(-1), right = endpoint(1);
	const bool ok_left = usable(left), ok_right = usable(right);
	auto candidate = [&](const End &e, int side) { return tangent(lidar_points[e.index].x, lidar_points[e.index].y, side); };
	auto path_length = [&](const End &e, int side) {
		auto [cx, cy] = candidate(e, side);
		return std::hypot(cx, cy) + std::hypot(gx - cx, gy - cy);
	};
	const int side = ok_left && ok_right ? (path_length(left, -1) <= path_length(right, 1) ? -1 : 1)
	               : ok_left ? -1 : ok_right ? 1 : 0;
	if (side == 0)
		return {gx, gy};   // muro sin extremo a la vista por ningun lado: lo lleva el DWA
	const End &e = side < 0 ? left : right;
	// Extremo a menos de r: el robot ya esta en la punta y el DWA la dobla solo (la meta esta justo detras).
	// Empezar aqui daba la submeta a la espalda, giro en el sitio y el rodeo se encendia y apagaba.
	if (std::hypot(lidar_points[e.index].x, lidar_points[e.index].y) < r)
		return {gx, gy};
	const auto sub = candidate(e, side);
	if (lidar_debug)
		std::cout << "[rodeo] tramo de " << blocking->size() << " sectores corta la recta a la meta a " << blocking_along
		          << " m (lateral " << nav_wall_cut_lateral << "); extremo " << (side < 0 ? "izquierdo" : "derecho")
		          << " (" << lidar_points[e.index].x << ", " << lidar_points[e.index].y << ") a "
		          << std::hypot(lidar_points[e.index].x, lidar_points[e.index].y) << " m -> submeta (" << sub.first
		          << ", " << sub.second << "); meta a " << goal_dist << " m" << (have_pose ? "" : " (sin pose: sin estado)") << std::endl;
	nav_wall_since = now;
	nav_wall_cycles = 1;
	nav_wall_cut_along = blocking_along;
	nav_wall_end_dist = std::hypot(lidar_points[e.index].x, lidar_points[e.index].y);
	if (have_pose)
	{
		nav_wall_side = side;
		nav_wall_switches = 0;
		nav_wall_hit_dist = nav_wall_best_dist = goal_dist;
		nav_wall_best_time = now;
		nav_wall_free = false;
		std::tie(nav_wall_goal_wx, nav_wall_goal_wy) = to_world(nav_target_x, nav_target_y);
		store_corner(e);
	}
	return sub;
}

std::pair<float, float> SpecificWorker::navigate_to(const DSR::Node &robot_node, float gx, float gy, float max_v, float max_w)
{
	// DWA con los costes de dwa.pdf: rumbo, distancia, holgura (x3, cuadratica hasta 0.5 m), velocidad,
	// espacio libre al final del arco, mismo lado al pasar un obstaculo y cambio de giro; freno por
	// holgura dentro del muestreo y regla de progreso.
	auto obstacles = collect_obstacles(robot_node);
	if (nav_debug_view)
	{
		nav_dbg = NavDebug{};
		nav_dbg.navigating = true;
		nav_dbg.goal = {gx, gy};
		nav_dbg.dsr = obstacles;
	}
	// Origen de la meta final, para la traza [giro]: la del objetivo, submeta del DSR o del rodeo de muros.
	const float goal_x0 = gx, goal_y0 = gy;
	std::tie(gx, gy) = detour_subgoal(gx, gy, obstacles);
	const float dsr_x = gx, dsr_y = gy;
	std::tie(gx, gy) = wall_detour_subgoal(gx, gy);
	const char *goal_source = (gx != dsr_x || gy != dsr_y) ? "submeta del rodeo de muros"
	                        : (gx != goal_x0 || gy != goal_y0) ? "submeta del DSR" : "objetivo";
	if (nav_debug_view)
		nav_dbg.subgoal = {gx, gy};
	const size_t n_dsr = obstacles.size();   // los de despues son del lidar, en el orden de lidar_points
	for (const auto &p : lidar_points)
		obstacles.push_back({p.x, p.y, NAV_ROBOT_RADIUS});

	// Meta de lado o detras: primero se encara girando en el sitio. Los arcos solo avanzan, y con
	// la meta a la espalda todos alejan antes de acercar. Histeresis para no alternar en el borde.
	const float goal_angle = std::atan2(gx, gy);
	if (std::abs(goal_angle) > (nav_rotating ? NAV_ROTATE_DONE_ANGLE : NAV_ROTATE_FIRST_ANGLE))
	{
		const float w_turn = turn_towards(goal_angle, max_w);
		if (!nav_rotating)
		{
			nav_rotate_since = std::chrono::steady_clock::now();
			if (lidar_debug)
				std::cout << "[giro] empieza en el sitio: meta a " << goal_angle * 180.f / std::numbers::pi_v<float>
				          << " grados (" << goal_source << ", " << std::hypot(gx, gy) << " m); objetivo a "
				          << std::atan2(goal_x0, goal_y0) * 180.f / std::numbers::pi_v<float> << " grados, "
				          << std::hypot(goal_x0, goal_y0) << " m; w " << w_turn << ", sentido bloqueado " << nav_turn_sign << std::endl;
		}
		nav_rotating = true;
		if (nav_debug_view)
		{
			nav_dbg.rotating = true;
			nav_dbg.w = w_turn;
		}
		return {0.f, w_turn};
	}
	if (nav_rotating && lidar_debug)
		std::cout << "[giro] fin tras " << std::chrono::duration<float>(std::chrono::steady_clock::now() - nav_rotate_since).count()
		          << " s: meta a " << goal_angle * 180.f / std::numbers::pi_v<float> << " grados (" << goal_source << ")" << std::endl;
	nav_rotating = false;

	// Al avanzar no se usan giros por encima de NAV_MAX_W, y solo a NAV_W_WINDOW del actual (en el
	// robot real, saltar de un extremo al otro hace que las ruedas se peleen).
	max_w = std::min(max_w, NAV_MAX_W);

	// Velocidad actual: la ultima mandada. Frenar es inmediato; acelerar, limitado.
	const auto current = getVelocitiesFromDSR();
	const float v_now = current[0] / 1000.f;
	const float w_now = current[1];
	auto now = std::chrono::steady_clock::now();
	const float dt = std::clamp(std::chrono::duration<float>(now - nav_last_time).count(), 0.01f, 0.2f);
	nav_last_time = now;
	// Velocidades objetivo en [0, max_v]; el arco simula la rampa desde v_now (frenar es inmediato) y
	// lo que se manda es esa rampa hacia el objetivo elegido. Con la ventana limitada a v_now + a*dt,
	// parado junto a un obstaculo ningun arco compensaba acercarse 3 cm y el robot no arrancaba.
	const float v_hi = max_v;
	const float w_mid = std::clamp(w_now, -max_w, max_w);
	const float w_lo = std::max(-max_w, w_mid - NAV_W_WINDOW), w_top = std::min(max_w, w_mid + NAV_W_WINDOW);
	auto w_of = [&](int j) { return w_lo + (w_top - w_lo) * j / (NAV_W_SAMPLES - 1); };

	// Distancia libre de partida a cada obstaculo: si ya se esta dentro de su radio, se aceptan los
	// arcos que no entran mas, para poder salir.
	std::vector<float> start_clear;
	for (const auto &o : obstacles)
		start_clear.push_back(std::hypot(o.x, o.y) - o.radius);

	const int steps = static_cast<int>(NAV_SIM_TIME / NAV_SIM_STEP);
	// Regla de progreso: lejos de la meta, un arco lento solo se coge si no hay uno rapido libre.
	const bool far = std::hypot(gx, gy) > NAV_PROGRESS_GOAL_DIST;
	struct Choice { float cost = std::numeric_limits<float>::max(); float v = 0.f, w = 0.f; int side = 0; int arc = -1; float clear = -1.f; };
	Choice fast, slow;

	// Diagnostico: arcos a v_max, que obstaculo invalida cada uno (-1 libre) o le pone la menor holgura.
	std::vector<int> top_block(NAV_W_SAMPLES, -1), top_closest(NAV_W_SAMPLES, -1);
	std::vector<float> top_clear(NAV_W_SAMPLES, std::numeric_limits<float>::max());
	std::vector<char> top_braked(NAV_W_SAMPLES, 0);

	std::vector<size_t> near;   // obstaculos al alcance del arco y de su rayo libre final
	for (int i = 0; i < NAV_V_SAMPLES; ++i)
	{
		const float v = v_hi * i / (NAV_V_SAMPLES - 1);
		near.clear();
		for (size_t n = 0; n < obstacles.size(); ++n)
			if (start_clear[n] < v * NAV_SIM_TIME + NAV_FREE_AHEAD)
				near.push_back(n);
		for (int j = 0; j < NAV_W_SAMPLES; ++j)
		{
			const float w = w_of(j);

			// Arco a w constante y avance en rampa hacia v, en el marco del robot: avance en +y, giro
			// positivo a la izquierda.
			float px = 0.f, py = 0.f, th = 0.f, vk = v_now;
			NavDebugArc *dbg_arc = nullptr;
			if (nav_debug_view)
			{
				nav_dbg.arcs.push_back({{{0.f, 0.f}}, 2});
				dbg_arc = &nav_dbg.arcs.back();
			}
			float min_clear = std::numeric_limits<float>::max();     // a cualquier obstaculo
			int min_clear_n = -1, block_n = -1, min_side = 0;
			bool valid = true;
			bool braked = false;
			for (int k = 0; k < steps && valid; ++k)
			{
				vk = v >= vk ? std::min(v, vk + NAV_ACCEL_V * NAV_SIM_STEP) : v;
				th += w * NAV_SIM_STEP;
				px += -std::sin(th) * vk * NAV_SIM_STEP;
				py +=  std::cos(th) * vk * NAV_SIM_STEP;
				if (dbg_arc && (k % NAV_DEBUG_ARC_STRIDE == NAV_DEBUG_ARC_STRIDE - 1 || k == steps - 1))
					dbg_arc->pts.emplace_back(px, py);
				for (const size_t n : near)
				{
					const float rx = obstacles[n].x - px, ry = obstacles[n].y - py;
					const float clear = std::sqrt(rx * rx + ry * ry) - obstacles[n].radius;
					if ((start_clear[n] < 0.f && clear < start_clear[n] - 0.01f) || (start_clear[n] >= 0.f && clear < 0.f))
					{
						valid = false;
						block_n = static_cast<int>(n);
						break;
					}
					// Freno: acercandose, a la velocidad de ese paso tiene que poder parar (NAV_BRAKE_DECEL)
					// antes del obstaculo.
					if (clear < start_clear[n] - 1e-3f && vk > std::sqrt(2.f * NAV_BRAKE_DECEL * std::max(clear, 0.f)))
					{
						valid = false;
						braked = true;
						block_n = static_cast<int>(n);
						break;
					}
					if (clear < min_clear)
					{
						min_clear = clear;
						min_clear_n = static_cast<int>(n);
						// Lado del robot por el que queda: derecha (+1) o izquierda (-1) de su rumbo
						min_side = rx * std::cos(th) + ry * std::sin(th) > 0.f ? 1 : -1;
					}
				}
			}
			if (i == NAV_V_SAMPLES - 1)
			{
				top_block[j] = block_n;
				top_closest[j] = min_clear_n;
				top_clear[j] = min_clear;
				top_braked[j] = braked;
			}
			if (dbg_arc)
			{
				if (!valid)
					dbg_arc->pts.emplace_back(px, py);   // hasta donde choca
				dbg_arc->status = !valid ? (braked ? 1 : 0) : 2;
				nav_dbg.n_valid += valid;
			}
			if (!valid)
				continue;

			// Espacio libre al final del arco, en linea recta segun su rumbo final.
			const float hx = -std::sin(th), hy = std::cos(th);
			float free = NAV_FREE_AHEAD;
			for (const size_t n : near)
			{
				const float rx = obstacles[n].x - px, ry = obstacles[n].y - py;
				const float along = rx * hx + ry * hy;
				const float lat = std::abs(rx * hy - ry * hx);
				const float r = obstacles[n].radius;
				if (along > 0.f && lat < r)
					free = std::min(free, std::max(0.f, along - std::sqrt(r * r - lat * lat)));
			}

			const float goal_dist = std::hypot(gx - px, gy - py);
			const float heading_err = std::abs(std::remainder(std::atan2(gx - px, gy - py) + th, 2.f * std::numbers::pi_v<float>));
			const float c_clear = min_clear >= NAV_SAFE_CLEARANCE ? 0.f
			                    : std::pow(1.f - std::max(min_clear, 0.f) / NAV_SAFE_CLEARANCE, 2.f);
			const int side = min_clear < NAV_SAFE_CLEARANCE ? min_side : 0;
			const float c_side = nav_pass_side != 0 && side != 0 && side != nav_pass_side ? 1.f : 0.f;

			const float cost = NAV_W_HEADING * heading_err / std::numbers::pi_v<float>
			                 + NAV_W_DIST * goal_dist / NAV_DIST_NORM
			                 + NAV_W_CLEARANCE * c_clear
			                 + NAV_W_SPEED * (1.f - v / std::max(max_v, 1e-3f))
			                 + NAV_W_FREE * (1.f - free / NAV_FREE_AHEAD)
			                 + NAV_W_SIDE * c_side
			                 + NAV_W_SMOOTH * std::abs(w - w_now) / (2.f * NAV_W_WINDOW);
			Choice &slot = far && v < NAV_PROGRESS_MIN_V ? slow : fast;
			if (cost < slot.cost)
				slot = {cost, v, w, side, static_cast<int>(nav_dbg.arcs.size()) - 1, min_clear};
		}
	}

	const Choice &chosen = fast.cost < std::numeric_limits<float>::max() ? fast : slow;
	const bool none = chosen.cost == std::numeric_limits<float>::max();
	// Se manda la rampa hacia el objetivo elegido; frenar, de golpe.
	const float v_cmd = chosen.v >= v_now ? std::min(chosen.v, v_now + NAV_ACCEL_V * dt) : chosen.v;
	const std::pair<float, float> best = {v_cmd, chosen.w};
	nav_pass_side = none ? 0 : chosen.side;
	if (nav_debug_view)
	{
		nav_dbg.chosen = none ? -1 : chosen.arc;
		nav_dbg.v_target = chosen.v;
		nav_dbg.v_cmd = v_cmd;
		nav_dbg.w = chosen.w;
		nav_dbg.min_clear = chosen.clear;
	}

	if (none && !nav_blocked_warned)
	{
		std::cerr << "navigate_to: ningun arco libre; robot parado." << std::endl;
		nav_blocked_warned = true;
	}
	else if (!none)
		nav_blocked_warned = false;

	// Traza: avance recortado con margen para ir rapido. Del arco a v_hi con el giro elegido, el
	// obstaculo que lo invalida (choque o no poder frenar) o, si es valido, el que le pone la menor holgura.
	const bool cut = v_hi >= NAV_CUT_TRACE_MIN_V && chosen.v < NAV_CUT_TRACE_FRACTION * v_hi;
	if (lidar_debug && cut && !nav_cut_active)
	{
		int j = 0;
		for (int k = 1; k < NAV_W_SAMPLES; ++k)
			if (std::abs(w_of(k) - best.second) < std::abs(w_of(j) - best.second))
				j = k;
		const int n = top_block[j] >= 0 ? top_block[j] : top_closest[j];
		std::cout << "[dwa] avance recortado: v_max=" << v_hi << " -> objetivo " << chosen.v << " m/s (mandado " << best.first
		          << "), w=" << best.second
		          << " | arco a v_hi " << (top_braked[j] ? "sin distancia de frenado"
		                                  : top_block[j] >= 0 ? "invalido"
		                                  : "valido, holgura " + std::to_string(top_clear[j]) + " m");
		if (n < 0)
			std::cout << ", sin obstaculo (coste)";
		else if (static_cast<size_t>(n) >= n_dsr)
		{
			const auto &p = lidar_points[n - n_dsr];
			std::cout << ", por punto " << p.src << " (" << p.x << ", " << p.y << ", z=" << p.z << ") a "
			          << std::hypot(p.x, p.y) << " m";
		}
		else
			std::cout << ", por nodo del DSR (" << obstacles[n].x << ", " << obstacles[n].y << ") radio "
			          << obstacles[n].radius << " m";
		std::cout << " | " << lidar_points.size() << " sectores del lidar" << std::endl;
	}
	else if (lidar_debug && !cut && nav_cut_active)
		std::cout << "[dwa] avance recuperado: objetivo " << chosen.v << " m/s" << std::endl;
	nav_cut_active = cut;

	return best;
}

void SpecificWorker::draw_nav_debug()
{
	// Vista cenital en el marco del robot (x derecha, y avance, robot mirando hacia arriba).
	const auto now = std::chrono::steady_clock::now();
	if (!nav_debug_label || std::chrono::duration<float>(now - nav_debug_last).count() < NAV_DEBUG_PERIOD)
		return;
	nav_debug_last = now;

	constexpr int W = NAV_DEBUG_W, H = NAV_DEBUG_H;
	constexpr float S = 110.f;                        // pixeles por metro
	const QPointF origin(W / 2.0, 450.0);   // hueco abajo para la leyenda
	auto P = [&](float x, float y) { return QPointF(origin.x() + x * S, origin.y() - y * S); };

	QImage img(W, H, QImage::Format_RGB32);
	img.fill(Qt::white);
	QPainter pt(&img);
	pt.setRenderHint(QPainter::Antialiasing);

	// Anillos cada metro y el alcance del lidar
	pt.setPen(QPen(QColor(235, 235, 235), 1));
	for (int r = 1; r <= 4; ++r)
		pt.drawEllipse(origin, r * S, r * S);
	pt.setPen(QPen(QColor(210, 210, 240), 1, Qt::DashLine));
	pt.drawEllipse(origin, lidar_max_range * S, lidar_max_range * S);

	const auto &d = nav_dbg;
	// Arcos: choca gris claro, no frena naranja claro, validos gris, elegido verde grueso
	if (d.navigating && !d.rotating)
		for (int pass = 0; pass < 3; ++pass)
			for (size_t a = 0; a < d.arcs.size(); ++a)
			{
				const auto &arc = d.arcs[a];
				const bool chosen = static_cast<int>(a) == d.chosen;
				if ((pass == 0 && (arc.status == 2 || chosen)) || (pass == 1 && (arc.status != 2 || chosen)) || (pass == 2 && !chosen))
					continue;
				QColor c = chosen ? QColor(0, 170, 0) : arc.status == 2 ? QColor(140, 140, 140)
				         : arc.status == 1 ? QColor(255, 200, 150) : QColor(215, 215, 215);
				pt.setPen(QPen(c, chosen ? 3.5 : 1));
				for (size_t k = 1; k < arc.pts.size(); ++k)
					pt.drawLine(P(arc.pts[k - 1].first, arc.pts[k - 1].second), P(arc.pts[k].first, arc.pts[k].second));
			}

	// Nodos del DSR: radio inflado (discontinuo) y su tamano real
	for (const auto &o : d.dsr)
	{
		pt.setPen(QPen(QColor(40, 90, 200), 1, Qt::DashLine));
		pt.setBrush(Qt::NoBrush);
		pt.drawEllipse(P(o.x, o.y), o.radius * S, o.radius * S);
		pt.setPen(Qt::NoPen);
		pt.setBrush(QColor(40, 90, 200));
		const float core = std::max(0.03f, o.radius - NAV_ROBOT_RADIUS);
		pt.drawEllipse(P(o.x, o.y), core * S, core * S);
	}

	// Puntos del lidar por tramo: muros en color, tramos cortos en gris, el que corta la recta en rojo
	static const QColor palette[] = {QColor(31, 119, 180), QColor(148, 103, 189), QColor(23, 190, 207),
	                                 QColor(140, 86, 75), QColor(188, 189, 34), QColor(227, 119, 194)};
	pt.setPen(Qt::NoPen);
	for (size_t i = 0; i < lidar_points.size(); ++i)
	{
		const int c = i < d.point_cluster.size() ? d.point_cluster[i] : -1;
		QColor col(90, 90, 90);
		if (c >= 0 && c == d.blocking_cluster)
			col = QColor(220, 30, 30);
		else if (c >= 0 && c < static_cast<int>(d.cluster_wall.size()) && d.cluster_wall[c])
			col = palette[c % 6];
		pt.setBrush(col);
		pt.drawEllipse(P(lidar_points[i].x, lidar_points[i].y), 3.0, 3.0);
	}

	// Objetivo y su exclusion
	if (lidar_target)
	{
		pt.setPen(QPen(QColor(200, 0, 200), 1, Qt::DashLine));
		pt.setBrush(Qt::NoBrush);
		pt.drawEllipse(P(lidar_target->first, lidar_target->second), lidar_target_exclusion * S, lidar_target_exclusion * S);
		pt.setPen(Qt::NoPen);
		pt.setBrush(QColor(200, 0, 200));
		pt.drawEllipse(P(lidar_target->first, lidar_target->second), 6.0, 6.0);
	}

	// Meta (azul) y submeta del rodeo (naranja, con su recta)
	auto cross = [&](std::pair<float, float> g, QColor c) {
		pt.setPen(QPen(c, 2.5));
		const QPointF q = P(g.first, g.second);
		pt.drawLine(q + QPointF(-7, -7), q + QPointF(7, 7));
		pt.drawLine(q + QPointF(-7, 7), q + QPointF(7, -7));
	};
	if (d.navigating)
	{
		cross(d.goal, QColor(0, 90, 255));
		if (std::hypot(d.subgoal.first - d.goal.first, d.subgoal.second - d.goal.second) > 1e-3f)
		{
			pt.setPen(QPen(QColor(255, 140, 0), 1.5, Qt::DashLine));
			pt.drawLine(P(0, 0), P(d.subgoal.first, d.subgoal.second));
			cross(d.subgoal, QColor(255, 140, 0));
		}
	}

	// Robot: radio de choque y rumbo
	pt.setPen(QPen(Qt::black, 2));
	pt.setBrush(Qt::NoBrush);
	pt.drawEllipse(origin, NAV_ROBOT_RADIUS * S, NAV_ROBOT_RADIUS * S);
	pt.drawLine(origin, P(0, NAV_ROBOT_RADIUS));

	// Texto
	pt.setPen(Qt::black);
	QFont font = pt.font();
	font.setPointSize(10);
	pt.setFont(font);
	QStringList lines;
	if (!d.navigating)
		lines << "sin navegar (afordance inactiva o ya a distancia)";
	else if (d.rotating)
		lines << QString("girando en el sitio: w=%1 rad/s").arg(d.w, 0, 'f', 2);
	else
	{
		lines << QString("v objetivo %1  mandada %2 m/s  w %3 rad/s").arg(d.v_target, 0, 'f', 2).arg(d.v_cmd, 0, 'f', 2).arg(d.w, 0, 'f', 2);
		lines << QString("arcos validos %1/%2  holgura elegido %3 m").arg(d.n_valid).arg(d.arcs.size())
		             .arg(d.min_clear > 1e3f ? QString("libre") : QString::number(d.min_clear, 'f', 2));
	}
	lines << QString("rodeo muro %1  rodeo DSR %2  lado de paso %3  freno %4")
	             .arg(nav_wall_side == 0 ? "no" : nav_wall_side < 0 ? "izq" : "der")
	             .arg(nav_detour_side == 0 ? "no" : nav_detour_side < 0 ? "izq" : "der")
	             .arg(nav_pass_side == 0 ? "-" : nav_pass_side < 0 ? "izq" : "der")
	             .arg(brake_active ? "ACTIVO" : "no");
	lines << QString("lidar %1 sectores").arg(lidar_points.size());
	for (int i = 0; i < lines.size(); ++i)
		pt.drawText(10, 20 + 16 * i, lines[i]);

	// Leyenda abajo, un elemento por linea, con su muestra de color
	struct LegendItem { QColor color; int kind; QString text; };   // kind: 0 linea, 1 punto, 2 cruz, 3 circulo discontinuo
	const std::vector<LegendItem> legend = {
		{QColor(215, 215, 215), 0, "arco que choca"},
		{QColor(255, 200, 150), 0, "arco que no puede frenar a tiempo"},
		{QColor(140, 140, 140), 0, "arco valido"},
		{QColor(0, 170, 0),     0, "arco elegido"},
		{QColor(31, 119, 180),  1, "lidar: muro (un color por tramo >= 0.5 m)"},
		{QColor(90, 90, 90),    1, "lidar: tramo corto (lo esquiva el DWA, no se rodea)"},
		{QColor(220, 30, 30),   1, "lidar: muro que corta la recta a la meta (se rodea)"},
		{QColor(40, 90, 200),   3, "nodo del DSR (discontinuo: radio inflado)"},
		{QColor(200, 0, 200),   1, "objetivo (RT del TARGET) y su exclusion del lidar"},
		{QColor(0, 90, 255),    2, "meta (a la distancia deseada del objetivo)"},
		{QColor(255, 140, 0),   2, "submeta del rodeo (extremo del muro o del nodo)"},
		{Qt::black,             3, "robot: radio de choque 0.40 m"}};
	const int y0 = H - 12 - 15 * static_cast<int>(legend.size() - 1);
	pt.fillRect(QRectF(4, y0 - 15, 380, 15 * legend.size() + 8), QColor(255, 255, 255, 225));   // fondo legible
	for (size_t i = 0; i < legend.size(); ++i)
	{
		const int y = y0 + 15 * static_cast<int>(i);
		const auto &it = legend[i];
		pt.setBrush(Qt::NoBrush);
		switch (it.kind)
		{
		case 0: pt.setPen(QPen(it.color, 3)); pt.drawLine(12, y - 4, 32, y - 4); break;
		case 1: pt.setPen(Qt::NoPen); pt.setBrush(it.color); pt.drawEllipse(QPointF(22, y - 4), 4, 4); break;
		case 2: pt.setPen(QPen(it.color, 2)); pt.drawLine(17, y - 9, 27, y + 1); pt.drawLine(17, y + 1, 27, y - 9); break;
		default: pt.setPen(QPen(it.color, 1.5, Qt::DashLine)); pt.drawEllipse(QPointF(22, y - 4), 6, 6); break;
		}
		pt.setPen(Qt::black);
		pt.drawText(40, y, it.text);
	}
	pt.end();

	nav_debug_label->setPixmap(QPixmap::fromImage(img));
	if (!nav_debug_save_dir.empty())
		img.save(QString::fromStdString(nav_debug_save_dir + "/" + std::to_string(
		         std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()) + ".png"));
	nav_dbg.navigating = false;   // si en el siguiente ciclo no se navega, se vera
	++nav_debug_frame;
}

void SpecificWorker::update_lidar_obstacles(const DSR::Node &robot_node)
{
	if (!lidar_enabled || !simulated)
		return;
	const auto now = std::chrono::steady_clock::now();
	if (std::chrono::duration<float>(now - lidar_last_read).count() < LIDAR_READ_PERIOD)
		return;
	lidar_last_read = now;

	// Objetivo en el marco del robot: sus puntos no son obstaculo (se le sigue o se le fotografia).
	std::optional<std::pair<float, float>> target;
	if (auto edges = G->get_edges_by_type("TARGET"); !edges.empty())
		if (auto t = rt->get_translation(robot_node, edges[0].to()); t.has_value()
		    && std::hypot(t->x(), t->y()) > TARGET_NULL_DISTANCE)
			target = {{static_cast<float>(t->x()), static_cast<float>(t->y())}};
	lidar_target = target;

	const auto bin_of = [](float x, float y) {
		const float angle = std::atan2(x, y) + std::numbers::pi_v<float>;
		return std::min(LIDAR_BINS - 1, static_cast<int>(angle / (2.f * std::numbers::pi_v<float>) * LIDAR_BINS));
	};
	const auto cell_key = [](int cx, int cy) { return (static_cast<int64_t>(cx) << 32) ^ static_cast<uint32_t>(cy); };

	std::optional<LidarPoint> low_front;
	bool got_data = false;
	int kept = 0;
	for (size_t m = 0; m < LIDAR_N_MOUNTS; ++m)
	{
		const auto &mount = LIDAR_MOUNTS[m];
		RoboCompLidar3D::TData data;
		try { data = lidar3d_proxy->getLidarData(mount.name, 0.f, 2.f * std::numbers::pi_v<float>, 1); }
		catch (const Ice::Exception &e)
		{
			if (!lidar_stale_warned)
				std::cerr << "Lidar " << mount.name << ": " << e.what() << std::endl;
			continue;
		}
		// El puente devuelve vacio si no hay muestra nueva desde la ultima lectura: se mantiene la anterior.
		if (data.points.empty())
			continue;
		got_data = true;
		// Por sector, el punto confirmado mas cercano: es el que limita, y deja el DWA en cientos de puntos.
		std::vector<LidarPoint> nearest(LIDAR_BINS, {0.f, 0.f, 0.f, ""});
		std::vector<float> nearest_d(LIDAR_BINS, std::numeric_limits<float>::max());
		std::unordered_set<int64_t> cells;
		auto &target_pts = lidar_mount_target_points[m];
		target_pts.clear();
		// Celdas de esta lectura con su numero de puntos validos (altura y alcance), para el aislamiento.
		std::unordered_map<int64_t, int> scan_cells;
		for (const auto &p : data.points)
		{
			const float x = p.y + mount.tx, y = p.x + mount.ty, z = -p.z + mount.tz;
			const float d = std::hypot(x, y);
			if (z >= lidar_min_height && z <= lidar_max_height && d >= lidar_self_radius && d <= lidar_max_range)
				++scan_cells[cell_key(static_cast<int>(std::floor(x / LIDAR_PERSIST_CELL)),
				                      static_cast<int>(std::floor(y / LIDAR_PERSIST_CELL)))];
		}
		for (const auto &p : data.points)
		{
			const float x = p.y + mount.tx, y = p.x + mount.ty, z = -p.z + mount.tz;
			const float d = std::hypot(x, y);
			if (z < lidar_min_height && d >= lidar_self_radius && std::abs(x) < LIDAR_LOW_CORRIDOR_HALF
			    && y > 0.f && y < LIDAR_LOW_CORRIDOR_LENGTH && (!low_front || z > low_front->z))
				low_front = LidarPoint{x, y, z, mount.name};
			if (z < lidar_min_height || z > lidar_max_height)
				continue;   // suelo y baches (los baches los lleva el DSR), o por encima del robot
			if (d < lidar_self_radius || d > lidar_max_range)
				continue;
			// Persistencia contra la lectura anterior de este lidar (celda propia o vecina).
			const int cx = static_cast<int>(std::floor(x / LIDAR_PERSIST_CELL));
			const int cy = static_cast<int>(std::floor(y / LIDAR_PERSIST_CELL));
			cells.insert(cell_key(cx, cy));
			// Aislamiento: el y al menos otro punto de esta lectura en su celda o las 8 vecinas.
			int around = 0;
			for (int dx = -1; dx <= 1 && around < 2; ++dx)
				for (int dy = -1; dy <= 1 && around < 2; ++dy)
					if (auto it = scan_cells.find(cell_key(cx + dx, cy + dy)); it != scan_cells.end())
						around += it->second;
			if (around < 2)
			{
				++lidar_isolated;
				if (!lidar_isolated_nearest || d < std::hypot(lidar_isolated_nearest->x, lidar_isolated_nearest->y))
					lidar_isolated_nearest = LidarPoint{x, y, z, mount.name};
				continue;
			}
			bool seen_before = false;
			for (int dx = -1; dx <= 1 && !seen_before; ++dx)
				for (int dy = -1; dy <= 1 && !seen_before; ++dy)
					seen_before = lidar_prev_cells[m].contains(cell_key(cx + dx, cy + dy));
			if (!seen_before)
			{
				++lidar_dropped;
				if (!lidar_dropped_nearest || d < std::hypot(lidar_dropped_nearest->x, lidar_dropped_nearest->y))
					lidar_dropped_nearest = LidarPoint{x, y, z, mount.name};
				continue;
			}
			// Junto al objetivo no es obstaculo, pero se guarda: la linea de vista lo necesita.
			if (target && std::hypot(x - target->first, y - target->second) < lidar_target_exclusion)
			{
				target_pts.push_back({x, y, z, mount.name});
				continue;
			}
			const int bin = bin_of(x, y);
			++kept;
			if (d < nearest_d[bin])
			{
				nearest_d[bin] = d;
				nearest[bin] = {x, y, z, mount.name};
			}
		}
		lidar_prev_cells[m] = std::move(cells);
		lidar_mount_points[m].clear();
		for (int b = 0; b < LIDAR_BINS; ++b)
			if (nearest_d[b] < std::numeric_limits<float>::max())
				lidar_mount_points[m].push_back(nearest[b]);
	}

	if (!got_data)
	{
		if (!lidar_stale_warned && std::chrono::duration<float>(now - lidar_last_data).count() > LIDAR_STALE_SECONDS)
		{
			std::cerr << "Lidar: sin datos nuevos en " << LIDAR_STALE_SECONDS << " s; se esquiva solo lo del DSR." << std::endl;
			lidar_stale_warned = true;
			lidar_points.clear();
			for (size_t m = 0; m < LIDAR_N_MOUNTS; ++m)
			{
				lidar_prev_cells[m].clear();
				lidar_mount_points[m].clear();
				lidar_mount_target_points[m].clear();
			}
		}
		return;
	}
	lidar_last_data = now;
	lidar_stale_warned = false;
	// Union de los dos lidars (el que no trae muestra nueva aporta sus ultimos confirmados), el mas
	// cercano por sector.
	std::vector<const LidarPoint *> merged(LIDAR_BINS, nullptr);
	for (const auto &points : lidar_mount_points)
		for (const auto &p : points)
		{
			const int bin = bin_of(p.x, p.y);
			if (!merged[bin] || std::hypot(p.x, p.y) < std::hypot(merged[bin]->x, merged[bin]->y))
				merged[bin] = &p;
		}
	lidar_points.clear();
	for (const auto *p : merged)
		if (p)
			lidar_points.push_back(*p);
	lidar_low_front = low_front;

	// Diagnostico: el punto mas cercano por cuadrante (con su z y su lidar), para comprobar el marco
	// contra la escena, y el descartado por bajo mas alto de delante, para ver a que altura sale el suelo.
	if (lidar_debug && std::chrono::duration<float>(now - lidar_last_debug).count() > LIDAR_DEBUG_PERIOD)
	{
		lidar_last_debug = now;
		const char *names[4] = {"delante", "derecha", "detras", "izquierda"};
		const LidarPoint *where[4] = {nullptr, nullptr, nullptr, nullptr};
		for (const auto &p : lidar_points)
		{
			const float a = std::atan2(p.x, p.y);   // 0 delante, positivo a la derecha
			const int q = std::abs(a) < std::numbers::pi_v<float> / 4 ? 0
			            : std::abs(a) > 3 * std::numbers::pi_v<float> / 4 ? 2 : (a > 0 ? 1 : 3);
			if (!where[q] || std::hypot(p.x, p.y) < std::hypot(where[q]->x, where[q]->y))
				where[q] = &p;
		}
		std::cout << "[lidar] " << kept << " puntos, " << lidar_points.size() << " sectores |";
		for (int q = 0; q < 4; ++q)
		{
			if (!where[q]) std::cout << " " << names[q] << " libre";
			else std::cout << " " << names[q] << " " << std::hypot(where[q]->x, where[q]->y) << " m (" << where[q]->x
			               << ", " << where[q]->y << ", z=" << where[q]->z << ", " << where[q]->src << ")";
		}
		if (lidar_low_front)
			std::cout << " | bajo delante: z max " << lidar_low_front->z << " en (" << lidar_low_front->x << ", "
			          << lidar_low_front->y << ", " << lidar_low_front->src << ")";
		// Descartados por persistencia desde la traza anterior, y el mas cercano de ellos.
		std::cout << " | sueltos: " << lidar_dropped;
		if (lidar_dropped_nearest)
			std::cout << ", el mas cercano a " << std::hypot(lidar_dropped_nearest->x, lidar_dropped_nearest->y) << " m ("
			          << lidar_dropped_nearest->x << ", " << lidar_dropped_nearest->y << ", z=" << lidar_dropped_nearest->z
			          << ", " << lidar_dropped_nearest->src << ")";
		std::cout << " | aislados: " << lidar_isolated;
		if (lidar_isolated_nearest)
			std::cout << ", el mas cercano a " << std::hypot(lidar_isolated_nearest->x, lidar_isolated_nearest->y) << " m ("
			          << lidar_isolated_nearest->x << ", " << lidar_isolated_nearest->y << ", z=" << lidar_isolated_nearest->z
			          << ", " << lidar_isolated_nearest->src << ")";
		std::cout << std::endl;
		lidar_dropped = 0;
		lidar_dropped_nearest.reset();
		lidar_isolated = 0;
		lidar_isolated_nearest.reset();
	}
}

bool SpecificWorker::brake_needed(float v, float w)
{
	// Mismo arco que el DWA, con el robot sin inflar y solo el tramo que se recorre antes de poder
	// parar. Cuenta solo si acerca a un punto: alejarse de algo ya pegado siempre se permite.
	const float horizon = std::min(brake_time + brake_margin / std::max(v, 1e-3f), 3.f);
	for (const auto &lp : lidar_points)
	{
		const float ox = lp.x, oy = lp.y;
		const float start = std::hypot(ox, oy);
		if (start > v * horizon + NAV_ROBOT_RADIUS)
			continue;   // fuera de alcance
		float px = 0.f, py = 0.f, th = 0.f;
		for (float t = BRAKE_SIM_STEP; t <= horizon + 1e-4f; t += BRAKE_SIM_STEP)
		{
			th += w * BRAKE_SIM_STEP;
			px += -std::sin(th) * v * BRAKE_SIM_STEP;
			py +=  std::cos(th) * v * BRAKE_SIM_STEP;
			const float d = std::hypot(px - ox, py - oy);
			if (d < NAV_ROBOT_RADIUS && d < start - 0.01f)
			{
				if (!brake_active)
					std::cout << "[freno] avance anulado: v=" << v << " m/s, w=" << w << " rad/s, punto ("
					          << ox << ", " << oy << ", z=" << lp.z << ", " << lp.src << ") a " << start << " m, choque en "
					          << t << " s" << std::endl;
				return true;
			}
		}
	}
	return false;
}

void SpecificWorker::apply_safety_brake()
{
	// Freno de seguridad: si el avance que se va a mandar lleva al robot contra un punto del lidar
	// en brake_time, se anula el avance (el giro se mantiene). En el DSR, para que lo grabado sea
	// lo que se manda. El DWA ya evita los puntos; esto cubre lo que aparece de golpe.
	auto robot_node_opt = G->get_node("robot");
	if (!robot_node_opt.has_value() || lidar_points.empty())
	{
		brake_active = false;
		return;
	}
	const auto velocities = getVelocitiesFromDSR();
	const float v = velocities[0] / 1000.f, w = velocities[1];
	const bool braking = v > 0.f && brake_needed(v, w);
	if (braking)
	{
		if (!brake_active)
			brake_start = std::chrono::steady_clock::now();
		DSR::Node robot_node = robot_node_opt.value();
		G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node, 0.f);
		G->update_node(robot_node);
	}
	else if (brake_active)
		std::cout << "[freno] liberado tras " << std::chrono::duration<float>(std::chrono::steady_clock::now() - brake_start).count()
		          << " s" << std::endl;
	brake_active = braking;
}

bool SpecificWorker::target_perceived()
{
	// concept_person marca perceived=false cuando la RT sale de su memoria y no de verla. Un objetivo
	// que no publica el atributo se da por percibido.
	auto target_node_opt = get_target_concept_node();
	if (!target_node_opt.has_value())
		return false;
	auto att = target_node_opt.value().attrs().find("perceived");
	if (att == target_node_opt.value().attrs().end())
		return true;
	try { return std::get<bool>(att->second.value()); }
	catch (const std::bad_variant_access&) { return true; }
}

void SpecificWorker::trace_person_ground_truth(const DSR::Node &robot_node)
{
	// Traza (simulacion): persona y robot en Webots (metros, marco de Webots), la persona real en el marco
	// del robot (x derecha, y avance) y la RT publicada, con su error. Hora en ms epoch, como las grabaciones.
	if (!simulated || !lidar_debug)
		return;
	const auto now = std::chrono::steady_clock::now();
	if (std::chrono::duration<float>(now - person_trace_last).count() < PERSON_TRACE_PERIOD)
		return;
	person_trace_last = now;
	RoboCompWebots2Robocomp::ObjectPose person_pose, robot_pose;
	try
	{
		person_pose = webots2robocomp_proxy->getObjectPose(person_DEF);
		robot_pose = webots2robocomp_proxy->getObjectPose(robot_DEF);
	}
	catch (const Ice::Exception &e)
	{
		if (!person_trace_warned)
			std::cerr << "[persona] sin pose de Webots: " << e.what() << std::endl;
		person_trace_warned = true;
		return;
	}
	person_trace_warned = false;
	const float PI = std::numbers::pi_v<float>;
	const float px = person_pose.position.x / 1000.f, py = person_pose.position.y / 1000.f;
	const float rx = robot_pose.position.x / 1000.f, ry = robot_pose.position.y / 1000.f;
	Eigen::Quaternionf q(robot_pose.orientation.w, robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z);
	q.normalize();
	const float yaw = std::atan2(2.f * (q.w() * q.z() + q.x() * q.y()), 1.f - 2.f * (q.y() * q.y() + q.z() * q.z()));
	// Marco del robot = marco local de Webots del Shadow (x derecha, y avance)
	const float dx = px - rx, dy = py - ry;
	const float real_x = std::cos(yaw) * dx + std::sin(yaw) * dy, real_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
	const auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count();
	std::cout << "[persona] " << epoch_ms << " webots persona (" << px << ", " << py << ") robot (" << rx << ", " << ry
	          << ", yaw " << yaw * 180.f / PI << ") w " << getVelocitiesFromDSR()[1] << " | real en el robot (" << real_x << ", "
	          << real_y << ")";
	if (auto person_node = G->get_node("person"); person_node.has_value())
	{
		bool perceived = true;
		if (auto att = person_node->attrs().find("perceived"); att != person_node->attrs().end())
			try { perceived = std::get<bool>(att->second.value()); } catch (const std::bad_variant_access &) {}
		if (auto t = rt->get_translation(robot_node, person_node->id()); t.has_value())
		{
			const float tx = static_cast<float>(t->x()), ty = static_cast<float>(t->y());
			const float angle_error = std::remainder(std::atan2(tx, ty) - std::atan2(real_x, real_y), 2.f * PI);
			std::cout << " | RT (" << tx << ", " << ty << ") perceived " << perceived << " | error "
			          << std::hypot(tx - real_x, ty - real_y) << " m, " << angle_error * 180.f / PI << " grados";
		}
		else
			std::cout << " | sin RT robot->persona";
	}
	std::cout << std::endl;
}

std::vector<float> SpecificWorker::auto_localization()
{
	std::vector<float> robot_pose = {0,0,0,0,0,0,1}; // {x, y, z, qx, qy, qz, qw}
	if (simulated){
		auto webots_pose = this->webots2robocomp_proxy->getObjectPose(robot_DEF);
		robot_pose[0] = webots_pose.position.y / 1000.f;
		robot_pose[1] = webots_pose.position.x / 1000.f;
		robot_pose[2] = webots_pose.position.z / 1000.f;
		Eigen::Quaternionf quat(webots_pose.orientation.w, webots_pose.orientation.x, webots_pose.orientation.y, webots_pose.orientation.z);
		quat.normalize();
		robot_pose[3] = quat.x();
		robot_pose[4] = quat.y();
		robot_pose[5] = quat.z();
		robot_pose[6] = quat.w();
	}
	else{
		if (last_odometry.empty())
			return robot_pose;
			
		robot_pose[0] = last_odometry[0];
		robot_pose[1] = last_odometry[1];
		robot_pose[2] = 0.0f;
		Eigen::AngleAxisf rot_z(last_odometry[2], Eigen::Vector3f::UnitZ());
		Eigen::Quaternionf quat(rot_z);
		robot_pose[3] = quat.x();
		robot_pose[4] = quat.y();
		robot_pose[5] = quat.z();
		robot_pose[6] = quat.w();
	}

	if (!has_significant_change(robot_pose, last_robot_pose))
	{
		last_robot_pose = robot_pose;
		return robot_pose;
	}

	auto robot_node_opt = G->get_node("robot");
    auto root_node_opt = G->get_node("root");

	DSR::Node root_node, robot_node;

	if (!robot_node_opt.has_value())
	{
		std::cerr << "Robot node not found in DSR . Creating new robot node." << std::endl;
		robot_node = DSR::Node::create<robot_node_type>("robot");
		G->insert_node(robot_node);
	}
	else 
		robot_node = robot_node_opt.value();

	if (!root_node_opt.has_value())
	{
		std::cerr << "Root node not found in DSR. Creating new root node." << std::endl;
		root_node = DSR::Node::create<root_node_type>("root");
		G->insert_node(root_node);
	}
	else
		root_node = root_node_opt.value();

	auto rt_edge_opt = rt->get_edge_RT(root_node, robot_node.id());
	if (!rt_edge_opt.has_value())
	{
		std::cerr << "RT edge between root and robot not found. Creating new RT edge." << std::endl;
		DSR::Edge new_rt_edge;
		new_rt_edge.from(root_node.id());
		new_rt_edge.to(robot_node.id());
		new_rt_edge.type("RT");
		G->add_or_modify_attrib_local<rt_translation_att>(new_rt_edge, (std::vector<float>){robot_pose[0], robot_pose[1], robot_pose[2]});
		G->add_or_modify_attrib_local<rt_quaternion_att>(new_rt_edge, (std::vector<float>){robot_pose[3], robot_pose[4], robot_pose[5], robot_pose[6]});
		G->insert_or_assign_edge(new_rt_edge);
	}
	else
	{
		DSR::Edge rt_edge = rt_edge_opt.value();
		G->add_or_modify_attrib_local<rt_translation_att>(rt_edge, (std::vector<float>){robot_pose[0], robot_pose[1], robot_pose[2]});
		G->add_or_modify_attrib_local<rt_quaternion_att>(rt_edge, (std::vector<float>){robot_pose[3], robot_pose[4], robot_pose[5], robot_pose[6]});
		G->insert_or_assign_edge(rt_edge);
	}

	return robot_pose;
}

#pragma endregion ROBOT_METHODS

#pragma region DSR

std::vector<float> SpecificWorker::getVelocitiesFromDSR()
{
	std::vector<float> velocities = {0.0, 0.0}; // {advx, rot}

	//Access to DSR to get the desired velocities
	auto optional_robot_node = G->get_node("robot");
	if (optional_robot_node.has_value())
	{
		auto robot_node = optional_robot_node.value();
		auto optional_adv_speed = G->get_attrib_by_name<robot_ref_adv_speed_att>(robot_node.id());
		if (optional_adv_speed.has_value())
		{
			velocities[0] = optional_adv_speed.value() * 1000;
		}
		auto optional_rot_speed = G->get_attrib_by_name<robot_ref_rot_speed_att>(robot_node.id());
		if (optional_rot_speed.has_value())
		{
			velocities[1] = optional_rot_speed.value();
		}
	}

	return velocities;
}

bool SpecificWorker::has_significant_change(const std::vector<float>& a,
                                            const std::vector<float>& b,
                                            double atol)
{
    if (a.size() != b.size())
        return true;

    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > atol) {
            return true; 
        }
    }
    return false;
}


void SpecificWorker::update_or_create_imu_node()
{
	std::vector<float> acceleration, angularVel;

	try{
		auto acceleration_raw = this->imu_proxy->getAcceleration();
		auto angularVel_raw = this->imu_proxy->getAngularVel();

		acceleration = {acceleration_raw.XAcc, acceleration_raw.YAcc, acceleration_raw.ZAcc};
		angularVel = {angularVel_raw.XGyr, angularVel_raw.YGyr, angularVel_raw.ZGyr};

	}catch(const Ice::Exception& ex)
    {
        std::cout <<"IMU proxy exception:"<< ex << std::endl;
        throw;
    }

	if (acceleration.empty() || angularVel.empty())
		return;	

	// Webots devuelve NaN hasta el primer step tras habilitar el sensor. Dejar pasar ese NaN
	// congela el nodo: last_*_measurement se queda en NaN y ninguna comparacion posterior
	// detecta cambios, asi que el nodo "imu" no vuelve a generar eventos.
	auto all_finite = [](const std::vector<float>& v) {
		return std::all_of(v.begin(), v.end(), [](float x){ return std::isfinite(x); });
	};
	if (!all_finite(acceleration) || !all_finite(angularVel))
	{
		if (!imu_nan_warned)
		{
			std::cerr << "Lectura de IMU no finita; se descarta hasta que el sensor dé datos válidos." << std::endl;
			imu_nan_warned = true;
		}
		return;
	}

	if (auto imu_node_opt = G->get_node("imu"); imu_node_opt.has_value())
	{
		auto imu_real_node = imu_node_opt.value();
		if(has_significant_change(last_acceleration_measurement, acceleration) 
		or has_significant_change(last_angular_velocity_measurement, angularVel)){
			G->add_or_modify_attrib_local<imu_accelerometer_att>(imu_real_node, acceleration);
			G->add_or_modify_attrib_local<imu_gyroscope_att>(imu_real_node, angularVel);
			G->update_node(imu_real_node);

			last_acceleration_measurement = acceleration;
			last_angular_velocity_measurement = angularVel;
		}
	}
	else
	{
		auto robot_node_opt = G->get_node("robot");
		if (!robot_node_opt.has_value())
		{
			std::cerr << "Robot node not found in DSR. Cannot create IMU node without robot node." << std::endl;
			return;
		}
		DSR::Node robot_node = robot_node_opt.value();
		
		std::cout << "Creating IMU node in DSR." << std::endl; 
		DSR::Node imu_node = DSR::Node::create<imu_node_type>("imu");
		auto pos_x = G->get_attrib_by_name<pos_x_att>(robot_node.id()).value();
		auto pos_y = G->get_attrib_by_name<pos_y_att>(robot_node.id()).value();
		auto level = G->get_attrib_by_name<level_att>(robot_node.id()).value();
		G->add_or_modify_attrib_local<parent_att>(imu_node, robot_node.id());
		G->add_or_modify_attrib_local<pos_x_att>(imu_node, pos_x+100);
		G->add_or_modify_attrib_local<pos_y_att>(imu_node, pos_y+100);
		G->add_or_modify_attrib_local<level_att>(imu_node, level+1);

		G->add_or_modify_attrib_local<imu_accelerometer_att>(imu_node, acceleration);
		G->add_or_modify_attrib_local<imu_gyroscope_att>(imu_node, angularVel);

		G->insert_node(imu_node);
		G->update_node(imu_node);
	
		DSR::Edge imu_edge;
		imu_edge.from(robot_node.id());
		imu_edge.to(imu_node.id());
		imu_edge.type("has");
		G->insert_or_assign_edge(imu_edge);
	
	}
}

bool SpecificWorker::check_affordance_active()
{
	auto target_edges = G->get_edges_by_type("TARGET");
	auto has_intention_edges = G->get_edges_by_type("has_intention");
	for (const auto& target_edge : target_edges)
	{
		for (const auto& intention_edge : has_intention_edges)
		{
			if (intention_edge.from() == target_edge.to())
			{
				auto affordance_node_opt = G->get_node(intention_edge.to());
				if (affordance_node_opt.has_value())
				{
					DSR::Node affordance_node = affordance_node_opt.value();
					bool aff_interacting = G->get_attrib_by_name<aff_interacting_att>(affordance_node.id()).value();
					
					return aff_interacting;
				}
			}
		}
	}
	return false;
}

void SpecificWorker::stop_robot()
{
	auto robot_node_opt = G->get_node("robot");
	if (!robot_node_opt.has_value())	{
		std::cerr << "Robot node not found in DSR. Cannot stop robot without robot node." << std::endl;
		return;
	}

	auto DSR_velocities = getVelocitiesFromDSR();
	if (DSR_velocities[0] == 0.0 && DSR_velocities[1] == 0.0){
		return; // Robot is already stopped in DSR, no need to update.
	}

	DSR::Node robot_node = robot_node_opt.value();
	G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node, (float)0.0);
	G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node, (float)0.0);
	G->update_node(robot_node);
}

#pragma endregion DSR

#pragma region PHOTO_MISSION

bool SpecificWorker::is_photo_target()
{
	auto target_edges = G->get_edges_by_type("TARGET");
	if (target_edges.empty())
		return false;
	// Mision de fotos: el concepto del TARGET tiene una afordance (has_intention) con aff_mission "take_photos"
	const uint64_t concept_id = target_edges[0].to();
	for (const auto& edge : G->get_edges_by_type("has_intention"))
	{
		if (edge.from() != concept_id)
			continue;
		auto affordance_opt = G->get_node(edge.to());
		if (!affordance_opt.has_value())
			continue;
		auto it = affordance_opt.value().attrs().find("aff_mission");
		if (it == affordance_opt.value().attrs().end())
			continue;
		try {
			if (std::get<std::string>(it->second.value()) == "take_photos")
				return true;
		} catch (const std::bad_variant_access&) {}
	}
	return false;
}

void SpecificWorker::reset_photo_spin()
{
	spin_stage = SpinStage::APPROACH;
	approach_started = false;
	last_target_distance = -1.f;
	last_target_angle = 0.f;
	spin_accumulated = 0.f;
	spin_since_shot = photo_angular_step;   // dispara ya en el rumbo de partida
	spin_heading_valid = false;
}

void SpecificWorker::spin_in_place(float angular_speed)
{
	auto robot_node_opt = G->get_node("robot");
	if (!robot_node_opt.has_value())
	{
		std::cerr << "Robot node not found in DSR." << std::endl;
		return;
	}
	DSR::Node robot_node = robot_node_opt.value();

	G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node, 0.0f);
	G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node, angular_speed);
	G->update_node(robot_node);
}

// Nodo al que apunta el TARGET: el concepto del que va esta mision.
std::optional<DSR::Node> SpecificWorker::get_target_concept_node()
{
	auto target_edges = G->get_edges_by_type("TARGET");
	if (target_edges.empty())
		return {};
	return G->get_node(target_edges[0].to());
}

// Pide la verificacion al agente del concepto poniendo verify_request en su nodo.
void SpecificWorker::request_verification()
{
	auto concept_node_opt = get_target_concept_node();
	if (!concept_node_opt.has_value())
		return;
	DSR::Node concept_node = concept_node_opt.value();
	G->runtime_checked_add_or_modify_attrib_local(concept_node, "verify_request", true);
	G->update_node(concept_node);
}

// Veredicto del agente: "ok", "fail", o vacio si todavia no ha contestado.
std::string SpecificWorker::read_verify_result()
{
	auto concept_node_opt = get_target_concept_node();
	if (!concept_node_opt.has_value())
		return "";
	DSR::Node concept_node = concept_node_opt.value();
	auto att = concept_node.attrs().find("verify_result");
	if (att == concept_node.attrs().end())
		return "";
	try { return std::get<std::string>(att->second.value()); }
	catch (const std::bad_variant_access&) { return ""; }
}

// Distancia al TARGET actual. Solo el modulo: no depende de como esten orientados los ejes.
float SpecificWorker::target_distance()
{
	auto robot_node_opt = G->get_node("robot");
	auto target_edges = G->get_edges_by_type("TARGET");
	if (!robot_node_opt.has_value() || target_edges.empty())
		return -1.f;

	auto t_opt = rt->get_translation(robot_node_opt.value(), target_edges[0].to());
	if (!t_opt.has_value())
		return -1.f;

	return static_cast<float>(std::hypot(t_opt.value().x(), t_opt.value().y()));
}

// Pone aff_interacting=false en la afordance alcanzada por TARGET -> has_intention, que es como
// se le señala a mission_controller que la misión ha terminado.
void SpecificWorker::clear_target_affordance()
{
	auto target_edges = G->get_edges_by_type("TARGET");
	auto has_intention_edges = G->get_edges_by_type("has_intention");
	for (const auto& target_edge : target_edges)
		for (const auto& intention_edge : has_intention_edges)
			if (intention_edge.from() == target_edge.to())
			{
				auto affordance_node_opt = G->get_node(intention_edge.to());
				if (affordance_node_opt.has_value())
				{
					DSR::Node affordance_node = affordance_node_opt.value();
					G->add_or_modify_attrib_local<aff_interacting_att>(affordance_node, false);
					G->update_node(affordance_node);
					return;
				}
			}
}

// Da follow_person por terminada tras aguantar follow_hold_seconds a la distancia deseada;
// alejarse o derivar durante la espera reinicia la cuenta.
std::optional<SpecificWorker::LidarPoint> SpecificWorker::wall_between_robot_and_target()
{
	// Puntos del lidar (los obstaculos y los que se excluyen junto al objetivo) agrupados por angulo; un
	// tramo de muro (>= FOLLOW_LOS_WALL_LENGTH) tapa si cruza la recta al objetivo antes de llegar a el,
	// o si el objetivo esta encima del tramo.
	if (!lidar_enabled || !simulated || !lidar_target)
		return std::nullopt;
	std::vector<LidarPoint> pts = lidar_points;
	for (const auto &mount_pts : lidar_mount_target_points)
		pts.insert(pts.end(), mount_pts.begin(), mount_pts.end());
	if (pts.size() < 2)
		return std::nullopt;
	std::sort(pts.begin(), pts.end(), [](const LidarPoint &a, const LidarPoint &b) {
		return std::atan2(a.x, a.y) < std::atan2(b.x, b.y); });

	const auto [tx, ty] = lidar_target.value();
	const float d = std::hypot(tx, ty);
	if (d < 1e-3f)
		return std::nullopt;
	const float ux = tx / d, uy = ty / d;

	size_t start = 0;
	for (size_t i = 1; i <= pts.size(); ++i)
	{
		if (i < pts.size() && std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y) < LIDAR_POINT_GAP)
			continue;
		// Tramo [start, i)
		float min_x = std::numeric_limits<float>::max(), max_x = -min_x, min_y = min_x, max_y = -min_x;
		for (size_t k = start; k < i; ++k)
		{
			min_x = std::min(min_x, pts[k].x); max_x = std::max(max_x, pts[k].x);
			min_y = std::min(min_y, pts[k].y); max_y = std::max(max_y, pts[k].y);
		}
		if (std::hypot(max_x - min_x, max_y - min_y) >= FOLLOW_LOS_WALL_LENGTH)
		{
			// Objetivo encima del muro: hay un punto a menos de FOLLOW_LOS_TARGET_ON_WALL de su RT y el tramo
			// sigue a los dos lados de la linea de vista (FOLLOW_LOS_SIDE_SPAN). Una persona junto a la punta de
			// un muro (sus puntos se juntan con el) solo lo tiene a un lado: antes bloqueaba el cierre.
			const LidarPoint *on_target = nullptr;
			bool left = false, right = false;
			for (size_t k = start; k < i; ++k)
			{
				const float along = pts[k].x * ux + pts[k].y * uy;
				const float lat = pts[k].x * uy - pts[k].y * ux;
				if (along > 0.f && along < d - FOLLOW_LOS_TARGET_MARGIN && std::abs(lat) < FOLLOW_LOS_HALF_WIDTH)
				{
					follow_los_reason = "el muro cruza la recta al objetivo";
					return pts[k];
				}
				if (std::hypot(pts[k].x - tx, pts[k].y - ty) < FOLLOW_LOS_TARGET_ON_WALL)
					on_target = &pts[k];
				if (std::abs(along - d) < FOLLOW_LOS_SIDE_SPAN)
				{
					left = left || lat < -FOLLOW_LOS_SIDE_SPAN;
					right = right || lat > FOLLOW_LOS_SIDE_SPAN;
				}
			}
			if (on_target && left && right)
			{
				follow_los_reason = "el objetivo esta sobre un muro que sigue a los dos lados";
				return *on_target;
			}
		}
		start = i;
	}
	return std::nullopt;
}

void SpecificWorker::check_follow_reached()
{
	if (last_target_distance < 0.f)
		return;

	// Con la persona fuera de la vista la RT sale de la memoria de concept_person: llegar a donde
	// se la vio por ultima vez no es haberla alcanzado.
	if (!target_perceived())
	{
		follow_holding = false;
		return;
	}

	// Una medida congelada es indistinguible de un objetivo quieto a la distancia justa, y
	// completaria la mision sola: solo cuenta si follow_target la ha refrescado hace poco.
	if (std::chrono::duration<float>(std::chrono::steady_clock::now() - last_target_stamp).count()
	    > TARGET_MAX_AGE)
	{
		follow_holding = false;
		return;
	}

	// El controlador nunca retrocede, así que se acepta la distancia deseada o menos; el margen
	// cubre que los últimos centímetros se recorren a milímetros por segundo.
	if (last_target_distance > desired_distance + FOLLOW_REACHED_MARGIN)
	{
		follow_holding = false;
		return;
	}

	// Con un muro en medio no se ha alcanzado a la persona: la camara la ve por encima de los muros
	// bajos y su RT puede caer en el propio muro (la profundidad de lo que la tapa).
	if (const auto wall = wall_between_robot_and_target(); wall.has_value())
	{
		if (!follow_los_warned)
			std::cout << "follow_person: a la distancia deseada pero con un muro en medio (punto " << wall->src << " ("
			          << wall->x << ", " << wall->y << "), " << follow_los_reason << "); no se completa." << std::endl;
		follow_los_warned = true;
		follow_holding = false;
		return;
	}
	follow_los_warned = false;

	auto now = std::chrono::steady_clock::now();
	if (!follow_holding)
	{
		follow_holding = true;
		follow_hold_start = now;
		follow_hold_distance = last_target_distance;
		return;
	}

	// Deriva respecto a la distancia que había al empezar: el objetivo se ha movido.
	if (std::fabs(last_target_distance - follow_hold_distance) > FOLLOW_HOLD_DRIFT)
	{
		follow_holding = false;
		return;
	}

	if (std::chrono::duration<float>(now - follow_hold_start).count() >= follow_hold_seconds)
	{
		std::cout << "follow_person: " << follow_hold_seconds << " s a " << last_target_distance
		          << " m del objetivo; misión completada." << std::endl;
		clear_target_affordance();
		follow_holding = false;
		last_target_distance = -1.f;
	}
}

void SpecificWorker::photo_spin()
{
	auto robot_node_opt = G->get_node("robot");
	auto root_node_opt  = G->get_node("root");
	if (!robot_node_opt.has_value() || !root_node_opt.has_value())
		return;
	DSR::Node robot_node = robot_node_opt.value();

	// Rumbo del robot desde la RT root->robot, que auto_localization() refresca cada ciclo: es la
	// señal rápida con la que se mide el giro acumulado.
	auto root_robot_rt_opt = rt->get_edge_RT(root_node_opt.value(), robot_node.id());
	if (!root_robot_rt_opt.has_value())
		return;
	auto q_rr_opt = G->get_attrib_by_name<rt_quaternion_att>(root_robot_rt_opt.value());
	if (!q_rr_opt.has_value())
		return;
	std::vector<float> q_rr = q_rr_opt.value();
	Eigen::Quaternionf q(q_rr[3], q_rr[0], q_rr[1], q_rr[2]);
	q.normalize();
	float robot_heading = std::atan2(2.f * (q.w() * q.z() + q.x() * q.y()),
		1.f - 2.f * (q.y() * q.y() + q.z() * q.z()));

	const float PI = std::numbers::pi_v<float>;
	auto wrap = [PI](float a) { return std::atan2(std::sin(a), std::cos(a)); };

	switch (spin_stage)
	{
		case SpinStage::APPROACH:
		{
			// Colocarse a photo_approach_distance del concepto y mirandolo: follow_target hace las
			// dos cosas. Demasiado cerca, la meta le cae a la espalda y da la vuelta para alejarse.
			float d = target_distance();
			if (d < 0.f)
			{
				stop_robot();
				break;   // sin RT del concepto no hay nada que medir
			}

			auto now = std::chrono::steady_clock::now();
			if (!approach_started)
			{
				approach_started = true;
				approach_check_start = now;
				approach_start = now;
			}

			if (std::chrono::duration<float>(now - approach_start).count() > PHOTO_APPROACH_TIMEOUT)
			{
				std::cout << "photo_spin APPROACH: sin colocarse en " << PHOTO_APPROACH_TIMEOUT
				          << " s; se aborta." << std::endl;
				spin_stage = SpinStage::ABORTED;
				break;
			}

			if (std::chrono::duration<float>(now - approach_check_start).count() >= 0.5f)
			{
				std::cout << "photo_spin APPROACH: d=" << d << " m | angulo="
				          << last_target_angle * 180.f / std::numbers::pi_v<float> << " deg" << std::endl;
				approach_check_start = now;
			}

			// Sin una medida propia de follow_target no se puede dar por bueno nada.
			if (last_target_distance < 0.f
			    || std::abs(d - photo_approach_distance) > PHOTO_APPROACH_MARGIN
			    || std::abs(last_target_angle) > PHOTO_APPROACH_ANGLE_TOLERANCE)
			{
				follow_target(PHOTO_APPROACH_SPEED_FACTOR, PHOTO_APPROACH_ANGULAR_FACTOR,
				              photo_approach_distance, PHOTO_APPROACH_MARGIN);
				break;
			}

			std::cout << "photo_spin: colocado a " << d << " m del objetivo y mirandolo; pidiendo verificacion." << std::endl;
			stop_robot();
			request_verification();
			verify_wait_start = std::chrono::steady_clock::now();
			spin_stage = SpinStage::VERIFY;
			break;
		}

		case SpinStage::VERIFY:
		{
			// Quieto: la aproximacion ya ha dejado el concepto delante, asi que el agente lo tiene
			// en cuadro y contesta en el primer ciclo. Sin respuesta antes del tope, negativa.
			stop_robot();
			const std::string verdict = read_verify_result();
			if (verdict == "ok")
			{
				std::cout << "photo_spin: verificacion correcta; empieza el giro." << std::endl;
				stop_robot();
				spin_accumulated = 0.f;
				spin_since_shot = photo_angular_step;
				spin_heading_valid = false;
				spin_stage = SpinStage::TURNING;
			}
			else if (verdict == "fail" && force_photos)
			{
				// Force_photos: el veredicto queda grabado pero no frena el giro.
				std::cerr << "photo_spin: verificacion negativa; Force_photos activo, se hace el giro igual." << std::endl;
				stop_robot();
				spin_accumulated = 0.f;
				spin_since_shot = photo_angular_step;
				spin_heading_valid = false;
				spin_stage = SpinStage::TURNING;
			}
			else if (verdict == "fail")
			{
				std::cerr << "photo_spin: verificacion negativa." << std::endl;
				spin_stage = SpinStage::ABORTED;
			}
			else if (std::chrono::duration<float>(std::chrono::steady_clock::now() - verify_wait_start).count() > verify_timeout_seconds)
			{
				std::cerr << "photo_spin: el agente del concepto no contesto en " << verify_timeout_seconds << " s." << std::endl;
				spin_stage = SpinStage::ABORTED;
			}
			break;
		}

		case SpinStage::ABORTED:
		{
			// TODO: decidir que hacer con una verificacion negativa (reintentar desde otro punto,
			// pedir otra simulacion, marcar la causa como no verificable). Por ahora se cierra la
			// mision para no dejar al planificador esperando.
			stop_robot();
			clear_target_affordance();
			break;
		}

		case SpinStage::TURNING:
		{
			// Acumular el giro real hecho desde el ciclo anterior.
			if (spin_heading_valid)
			{
				float delta = std::abs(wrap(robot_heading - spin_last_heading));
				spin_accumulated += delta;
				spin_since_shot  += delta;
			}
			spin_last_heading = robot_heading;
			spin_heading_valid = true;

			if (spin_accumulated >= 2.f * PI)
			{
				stop_robot();
				spin_stage = SpinStage::DONE;
			}
			else if (spin_since_shot >= photo_angular_step)
			{
				stop_robot();
				spin_settle_start = std::chrono::steady_clock::now();
				spin_stage = SpinStage::SETTLING;
			}
			else
				spin_in_place(photo_spin_speed);
			break;
		}

		case SpinStage::SETTLING:
		{
			// Quieto photo_settle_seconds en cada paso: esa parada es la señal con la que el
			// agente del concepto sabe que puede capturar y etiquetar.
			stop_robot();
			if (std::chrono::duration<float>(std::chrono::steady_clock::now() - spin_settle_start).count() < photo_settle_seconds)
				break;

			spin_since_shot = 0.f;
			spin_stage = SpinStage::TURNING;
			break;
		}

		case SpinStage::DONE:
		{
			// Soltar la afordance cierra la misión, y es lo que el agente del concepto lee como
			// "ya están todas las fotos" para entrenar con ellas.
			stop_robot();
			clear_target_affordance();
			break;
		}
	}
}

#pragma endregion PHOTO_MISSION

//SUBSCRIPTION to newFullPose method from FullPoseEstimationPub interface
// void SpecificWorker::FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)
// {
// 	if (simulated)
// 		return;

//     if (!std::isfinite(pose.y) || !std::isfinite(pose.rz))
//     {
//         std::cerr << "[FullPoseEstimationPub_newFullPose] WARNING: invalid pose received "
//                   << "(y=" << pose.y << ", rz=" << pose.rz << "), skipping." << std::endl;
//         return;
//     }

// 	float adv, last_x, last_y, last_theta;
// 	adv = pose.y;

// 	if (last_odometry.empty())
// 	{
// 		last_x = 0;
// 		last_y = 0;
// 		last_theta = 0;
// 	}
// 	else
// 	{		
// 		last_x = last_odometry[0];
// 		last_y = last_odometry[1];
// 		last_theta = last_odometry[2];
// 	}

// 		float theta = last_theta + pose.rz;
// 		float x = adv * std::cos(theta) + last_x;
// 		float y = adv * std::sin(theta) + last_y;

// 		last_odometry = {x, y, theta};

// 		if (print_extra_info){
// 			std::cout << "[FullPoseEstimationPub_newFullPose]" << std::endl;
// 			std::cout << "  Raw pose    -> y (adv): " << adv 
// 					<< " | rz: " << pose.rz << std::endl;
// 			std::cout << "  Last odom   -> x: " << last_x 
// 					<< " | y: " << last_y 
// 					<< " | theta: " << last_theta << std::endl;
// 			std::cout << "  New odom    -> x: " << x 
// 					<< " | y: " << y 
// 					<< " | theta: " << theta << std::endl;
// 		}
// }

void SpecificWorker::FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)
{
    if (simulated)
        return;

    if (!std::isfinite(pose.vy) || !std::isfinite(pose.vx) || !std::isfinite(pose.vrz))
    {
        std::cerr << "[FullPoseEstimationPub_newFullPose] WARNING: invalid pose, skipping." << std::endl;
        return;
    }

    // Calcular dt a partir del timestamp de la pose (viene en ms)
    if (last_timestamp == 0)
    {
        last_timestamp = pose.timestamp;
        last_odometry = {0.f, 0.f, 0.f};
        return;
    }

    float dt = (pose.timestamp - last_timestamp) / 1000.f;
    last_timestamp = pose.timestamp;

    if (dt <= 0.f || dt > 1.f)
    {
        std::cerr << "[FullPoseEstimationPub_newFullPose] WARNING: anomalous dt=" << dt << ", skipping." << std::endl;
        return;
    }

    // Ventana deslizante de velocidades
    velocity_window.push_back({pose.vx, pose.vy, pose.vrz});
    if (velocity_window.size() > ODOMETRY_WINDOW_SIZE)
        velocity_window.pop_front();

    if (velocity_window.size() < ODOMETRY_WINDOW_SIZE)
        return;

    float avg_vx = 0.f, avg_vy = 0.f, avg_vrz = 0.f;
    for (const auto& [vx, vy, vrz] : velocity_window)
    {
        avg_vx  += vx;
        avg_vy  += vy;
        avg_vrz += vrz;
    }
    avg_vx  /= ODOMETRY_WINDOW_SIZE;
    avg_vy  /= ODOMETRY_WINDOW_SIZE;
    avg_vrz /= ODOMETRY_WINDOW_SIZE;

	if (std::abs(avg_vy) < LINEAR_VELOCITY_DEADBAND) avg_vy = 0.f;
	if (std::abs(avg_vx) < LINEAR_VELOCITY_DEADBAND) avg_vx = 0.f;
	if (std::abs(avg_vrz) < ANGULAR_VELOCITY_DEADBAND) avg_vrz = 0.f;

    float last_x     = last_odometry[0];
    float last_y     = last_odometry[1];
    float last_theta = last_odometry[2];

	float theta = last_theta + avg_vrz * dt;
	float x     = last_x + (avg_vy * std::sin(last_theta) + avg_vx * std::cos(last_theta)) * dt;
	float y     = last_y + (avg_vy * std::cos(last_theta) - avg_vx * std::sin(last_theta)) * dt;

    last_odometry = {x, y, theta};

    if (print_extra_info)
    {
        std::cout << "[FullPoseEstimationPub_newFullPose]\n"
                  << "  Raw vel    -> vy: " << pose.vy  << " | vx: " << pose.vx  << " | vrz: " << pose.vrz << "\n"
                  << "  Avg vel    -> vy: " << avg_vy   << " | vx: " << avg_vx   << " | vrz: " << avg_vrz  << "\n"
                  << "  dt         -> " << dt << " s\n"
                  << "  Last odom  -> x: " << last_x    << " | y: " << last_y    << " | theta: " << last_theta << "\n"
                  << "  New odom   -> x: " << x         << " | y: " << y         << " | theta: " << theta << std::endl;
    }
}



/**************************************/
// From the RoboCompCamera360RGB you can call this methods:
// RoboCompCamera360RGB::TImage this->camera360rgb_proxy->getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

/**************************************/
// From the RoboCompIMU you can call this methods:
// RoboCompIMU::Acceleration this->imu_proxy->getAcceleration()
// RoboCompIMU::Gyroscope this->imu_proxy->getAngularVel()
// RoboCompIMU::DataImu this->imu_proxy->getDataImu()
// RoboCompIMU::Magnetic this->imu_proxy->getMagneticFields()
// RoboCompIMU::Orientation this->imu_proxy->getOrientation()
// RoboCompIMU::void this->imu_proxy->resetImu()

/**************************************/
// From the RoboCompIMU you can use this types:
// RoboCompIMU::Acceleration
// RoboCompIMU::Gyroscope
// RoboCompIMU::Magnetic
// RoboCompIMU::Orientation
// RoboCompIMU::DataImu

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompWebots2Robocomp you can call this methods:
// RoboCompWebots2Robocomp::ObjectPose this->webots2robocomp_proxy->getObjectPose(string DEF)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->resetWebots()
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setDoorAngle(float angle)
// RoboCompWebots2Robocomp::void this->webots2robocomp_proxy->setPathToHuman(int humanId, RoboCompGridder::TPath path)

/**************************************/
// From the RoboCompWebots2Robocomp you can use this types:
// RoboCompWebots2Robocomp::Vector3
// RoboCompWebots2Robocomp::Quaternion
// RoboCompWebots2Robocomp::ObjectPose

