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

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
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

	rt = G->get_rt_api();

	// Prime last_relative_pose so the first has_significant_change() in compute() compares
	// against a real reading, not the {0,0,0} default. has_published_once (not this) is what
	// actually guarantees the first RT write happens.
	last_relative_pose = get_bump_relative_position();
}


void SpecificWorker::compute()
{
	auto bump_node_opt = G->get_node("bump");
	if (!bump_node_opt.has_value())
		return; // "bump" not created yet (semantic hasn't confirmed a cause) -> nothing to track.

	auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	auto relative_position = get_bump_relative_position();

	if (print_extra_info)
		std::cout << "[" << now_ms << "] get_bump_relative_position -> x: " << relative_position[0]
			<< " | y: " << relative_position[1] << " | z: " << relative_position[2] << std::endl;

	bool changed = !has_published_once || has_significant_change(relative_position, last_relative_pose);
	if (changed)
	{
		bool updated = update_relative_position_to_bump(relative_position, now_ms);
		if (updated)
		{
			last_relative_pose = relative_position;
			has_published_once = true;
		}
	}
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

/******* Geometric calculations ********/

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

std::vector<float> SpecificWorker::get_bump_relative_position()
{
	std::vector<float> relative_position = {0.0f, 0.0f, 0.0f};

	if (simulated){
		auto bump_pose  = this->webots2robocomp_proxy->getObjectPose(bump_def);
		auto robot_pose = this->webots2robocomp_proxy->getObjectPose(robot_def);

		float dx = (bump_pose.position.x - robot_pose.position.x) / 1000.f;
		float dy = (bump_pose.position.y - robot_pose.position.y) / 1000.f;
		float dz = (bump_pose.position.z - robot_pose.position.z) / 1000.f;

		Eigen::Quaternionf q(robot_pose.orientation.w, robot_pose.orientation.x,
							 robot_pose.orientation.y, robot_pose.orientation.z);
		q.normalize();
		float theta = std::atan2(2.f * (q.w() * q.z() + q.x() * q.y()), 1.f - 2.f * (q.y() * q.y() + q.z() * q.z()));
		float local_x = -std::sin(theta) * dx + std::cos(theta) * dy;
		float local_y = -std::cos(theta) * dx - std::sin(theta) * dy;

		relative_position = {local_x, local_y, dz};

		if (print_extra_info)
			std::cout << "local_delta -> dx: " << relative_position[0]
					  << " | dy: " << relative_position[1]
					  << " | theta: " << theta << std::endl;
	}
	else{
		// TODO: no trained detector for "bump" yet (ImageSegmentation's current model only
		// recognizes "bottle"/"person"). Wire this up once a real bump detector exists;
		// until then this branch returns {0,0,0} and never publishes on the real robot.
		auto segmented_objects = this->imagesegmentation_proxy->getSegmentedObjects(true, false);
		for (const auto& obj : segmented_objects){
			if (obj.label == "bump"){
				const auto& pc = obj.points3D;
				if (!pc.X.empty()){
					relative_position = {pc.X[0], pc.Y[0], pc.Z[0]};
				}
				break;
			}
		}
	}

	return relative_position;
}

bool SpecificWorker::update_relative_position_to_bump(const std::vector<float>& relative_position, std::uint64_t now_ms)
{
	auto optional_robot_node = G->get_node("robot");
	auto optional_bump_node  = G->get_node("bump");

	if (!optional_robot_node || !optional_bump_node)
	{
		std::cout << "Robot or bump node not found in DSR." << std::endl;
		return false;
	}

	if (relative_position.size() != 3)
	{
		std::cout << "Invalid relative position size." << std::endl;
		return false;
	}

	// Skip the jump filter entirely for the very first publish: there's no meaningful
	// "last" position to jump from yet, and requiring confirmations here would just delay
	// follow_target() seeing the target with no benefit (the bump can't have "jumped" from
	// nothing).
	if (has_published_once)
	{
		float dx = relative_position[0] - last_relative_pose[0];
		float dy = relative_position[1] - last_relative_pose[1];
		float dz = relative_position[2] - last_relative_pose[2];
		float jump_distance = std::sqrt(dx*dx + dy*dy + dz*dz);

		if (jump_distance > large_jump_threshold)
		{
			float pdx = relative_position[0] - pending_relative_position[0];
			float pdy = relative_position[1] - pending_relative_position[1];
			float pdz = relative_position[2] - pending_relative_position[2];
			float pending_distance = std::sqrt(pdx*pdx + pdy*pdy + pdz*pdz);

			if (pending_distance > large_jump_candidate_tolerance)
			{
				pending_relative_position = relative_position;
				consecutive_large_jump_confirmations = 1;
				if (print_extra_info)
					std::cout << "Large jump detected, waiting for confirmation (1/" << required_large_jump_confirmations << ")" << std::endl;
				return false;
			}
			else
			{
				consecutive_large_jump_confirmations++;
				if (print_extra_info)
					std::cout << "[" << now_ms << "] jump_distance: " << jump_distance
					<< " | threshold: " << large_jump_threshold
					<< " | confirmations: " << consecutive_large_jump_confirmations
					<< "/" << required_large_jump_confirmations << std::endl;

				if (consecutive_large_jump_confirmations < required_large_jump_confirmations)
					return false;
			}
		}
	}

	auto robot_node = optional_robot_node.value();
	auto bump_node  = optional_bump_node.value();
	rt->insert_or_assign_edge_RT(robot_node, bump_node.id(), relative_position, {0.0f, 0.0f, 0.0f});
	consecutive_large_jump_confirmations = 0;
	pending_relative_position = relative_position;
	return true;
}


//SUBSCRIPTION to setVisualObjects method from VisualElementsPub interface
void SpecificWorker::VisualElementsPub_setVisualObjects(RoboCompVisualElementsPub::TData data)
{
//subscribesToCODE

}



/**************************************/
// From the RoboCompImageSegmentation you can call this methods:
// RoboCompImageSegmentation::TData this->imagesegmentation_proxy->getAll(bool points3d, bool rgb)
// RoboCompImageSegmentation::TDepth this->imagesegmentation_proxy->getDepth()
// RoboCompImageSegmentation::TImage this->imagesegmentation_proxy->getImage()
// RoboCompImageSegmentation::ObjectList this->imagesegmentation_proxy->getSegmentedObjects(bool points3d, bool rgb)

/**************************************/
// From the RoboCompImageSegmentation you can use this types:
// RoboCompImageSegmentation::PointCloud
// RoboCompImageSegmentation::Polygon
// RoboCompImageSegmentation::SegmentedObject
// RoboCompImageSegmentation::TImage
// RoboCompImageSegmentation::TDepth
// RoboCompImageSegmentation::TData

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

/**************************************/
// From the RoboCompVisualElementsPub you can use this types:
// RoboCompVisualElementsPub::TObject
// RoboCompVisualElementsPub::TData
