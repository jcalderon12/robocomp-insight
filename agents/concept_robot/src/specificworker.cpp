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
	photo_angular_step  = configLoader.get<double>("Photo_angular_step") * deg2rad;
	photo_settle_seconds = configLoader.get<double>("Photo_settle_seconds");
	photo_spin_speed    = configLoader.get<double>("Photo_spin_speed_factor") * WEBOTS_MAX_ANGULAR_SPEED;

	std::cout << "Numeric locale active: " << setlocale(LC_NUMERIC, nullptr) << std::endl;
}



void SpecificWorker::compute()
{
	auto_localization();

    if (check_affordance_active())
	{
		// Si el TARGET es el bache, la misión es la de fotos; si no, seguimiento normal.
		if (is_photo_target())
			photo_spin();
		else
		{
			reset_photo_spin();
			follow_target(1.0f, 1.0f, desired_distance);
			check_follow_reached();
		}
	}
	else{
		stop_robot();
		nav_rotating = false;
		nav_turn_sign = 0;
		nav_detour_side = 0;
		follow_holding = false;
		last_target_distance = -1.f;
		reset_photo_spin();
	}

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

void SpecificWorker::follow_target(float max_forward_speed_factor, float max_angular_speed_factor, float desired_distance, float distance_tolerance)
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
        std::tie(linear_velocity, angular_velocity) = navigate_to(robot_node, x * scale, y * scale, max_v, max_w);
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
	if (std::abs(angle) < NAV_TURN_UNLOCK_ANGLE)
		nav_turn_sign = 0;
	else if (nav_turn_sign == 0 && std::abs(angle) > NAV_TURN_LOCK_ANGLE)
		nav_turn_sign = angle > 0.f ? 1 : -1;

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
		obstacles.push_back({ox, oy, size / 2.f + NAV_ROBOT_RADIUS + NAV_SAFETY_MARGIN});
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
	}
	return candidate(nav_detour_side);
}

std::pair<float, float> SpecificWorker::navigate_to(const DSR::Node &robot_node, float gx, float gy, float max_v, float max_w)
{
	const auto obstacles = collect_obstacles(robot_node);
	std::tie(gx, gy) = detour_subgoal(gx, gy, obstacles);

	// Meta de lado o detras: primero se encara girando en el sitio. Los arcos solo avanzan, y con
	// la meta a la espalda todos alejan antes de acercar. Histeresis para no alternar en el borde.
	const float goal_angle = std::atan2(gx, gy);
	if (std::abs(goal_angle) > (nav_rotating ? NAV_ROTATE_DONE_ANGLE : NAV_ROTATE_FIRST_ANGLE))
	{
		nav_rotating = true;
		return {0.f, turn_towards(goal_angle, max_w)};
	}
	nav_rotating = false;

	// Velocidad actual: la ultima mandada. Frenar es inmediato; acelerar, limitado.
	const auto current = getVelocitiesFromDSR();
	const float v_now = current[0] / 1000.f;
	const float w_now = current[1];
	auto now = std::chrono::steady_clock::now();
	const float dt = std::clamp(std::chrono::duration<float>(now - nav_last_time).count(), 0.01f, 0.2f);
	nav_last_time = now;
	const float v_hi = std::clamp(v_now + NAV_ACCEL_V * dt, 0.f, max_v);

	// Distancia libre de partida a cada obstaculo: si ya se esta dentro de su margen, se aceptan los
	// arcos que no entran mas, para poder salir.
	std::vector<float> start_clear;
	for (const auto &o : obstacles)
		start_clear.push_back(std::hypot(o.x, o.y) - o.radius);

	const int steps = static_cast<int>(NAV_SIM_TIME / NAV_SIM_STEP);
	float best_cost = std::numeric_limits<float>::max();
	std::pair<float, float> best = {0.f, 0.f};

	for (int i = 0; i < NAV_V_SAMPLES; ++i)
	{
		const float v = v_hi * i / (NAV_V_SAMPLES - 1);
		for (int j = 0; j < NAV_W_SAMPLES; ++j)
		{
			const float w = -max_w + 2.f * max_w * j / (NAV_W_SAMPLES - 1);

			// Arco a (v, w) constantes en el marco del robot: avance en +y, giro positivo a la izquierda.
			float px = 0.f, py = 0.f, th = 0.f;
			float min_clear = std::numeric_limits<float>::max();
			bool valid = true;
			for (int k = 0; k < steps && valid; ++k)
			{
				th += w * NAV_SIM_STEP;
				px += -std::sin(th) * v * NAV_SIM_STEP;
				py +=  std::cos(th) * v * NAV_SIM_STEP;
				for (size_t n = 0; n < obstacles.size(); ++n)
				{
					const float clear = std::hypot(px - obstacles[n].x, py - obstacles[n].y) - obstacles[n].radius;
					if (start_clear[n] < 0.f)
					{
						if (clear < start_clear[n] - 0.01f)
							valid = false;
					}
					else if (clear < 0.f)
						valid = false;
					else
						min_clear = std::min(min_clear, clear);
				}
			}
			if (!valid)
				continue;

			// Rumbo a la meta desde el final del arco, relativo al rumbo final del robot.
			const float goal_dist = std::hypot(gx - px, gy - py);
			const float heading_err = std::abs(std::remainder(std::atan2(gx - px, gy - py) + th, 2.f * std::numbers::pi_v<float>));
			const float clear_cost = min_clear >= NAV_CLEARANCE_CAP ? 0.f : (NAV_CLEARANCE_CAP - min_clear) / NAV_CLEARANCE_CAP;

			const float cost = NAV_W_GOAL * goal_dist
			                 + NAV_W_HEADING * heading_err
			                 + NAV_W_CLEARANCE * clear_cost
			                 - NAV_W_SPEED * v / std::max(max_v, 1e-3f)
			                 + NAV_W_SMOOTH * std::abs(w - w_now);
			if (cost < best_cost)
			{
				best_cost = cost;
				best = {v, w};
			}
		}
	}

	if (best_cost == std::numeric_limits<float>::max() && !nav_blocked_warned)
	{
		std::cerr << "navigate_to: ningun arco libre; robot parado." << std::endl;
		nav_blocked_warned = true;
	}
	else if (best_cost < std::numeric_limits<float>::max())
		nav_blocked_warned = false;

	return best;
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
	auto target_node_opt = G->get_node(target_edges[0].to());
	return target_node_opt.has_value() && target_node_opt.value().name() == "bump";
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

