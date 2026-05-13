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

#define DEBUG 0

// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <nanoflann.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "PointsViewer.h"




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

	struct BottlePose {
		float x, y, z;           // Posición (mm)
		float qx, qy, qz, qw;    // Orientación (Quaternion)
		float height, width, depth; // Dimensiones (mm)
		float tilt_angle;        // Ángulo de inclinación respecto a la vertical (grados)
	};

	struct EulerAngles {
		double roll;
		double pitch;
		double yaw;
	};

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
	 * \brief Checks if the bottle is rrelated to the robot in the DSR graph.
	 * \return True if the bottle is related to the robot, false otherwise.
	 */
	bool check_bottle_related_robot();

	/**
	 * \brief Updates the pose of the bottle in the DSR graph.
	 * \param bottle_pose The pose of the bottle to be updated in the graph.
	 */
	void update_bottle_pose_in_dsr(const std::optional<BottlePose>& bottle_pose);

	/**
	 * \brief Changes the reference frame from the robot to the room in the DSR graph.
	 */
	void change_rt_from_robot_to_room();

	/**
	 * \brief Eliminates the RT edge from the robot to the bottle in the DSR graph.
	 */
	void eliminate_rt_from_robot_to_bottle();

	/**
	 * \brief Validates if the given segmented object is a bottle.
	 * \param obj The segmented object to validate.
	 * \return True if the object is a bottle, false otherwise.
	 */
	bool validate_bottle(const RoboCompImageSegmentation::SegmentedObject& obj);

	/**
	 * \brief Filters the point cloud to keep only points within a specified range.
	 * \param point_cloud The original point cloud to be filtered.
	 * \return A filtered point cloud containing only points within the specified range.
	 */
	RoboCompImageSegmentation::PointCloud filter_pointcloud(const RoboCompImageSegmentation::PointCloud& point_cloud);

	/**
	 * \brief Detects the bottle and returns its position as a vector of floats.
	 * \return A vector of floats representing the position of the detected bottle.
	 */
	std::optional<SpecificWorker::BottlePose> detect_bottle();

	/**
	 * \brief Checks if the bottle has fallen based on its orientation.
	 * \param pose The pose of the bottle.
	 * \return True if the bottle has fallen, false otherwise.
	 */
	bool bottle_has_fallen(const std::optional<BottlePose>& bottle_pose);

private:

	SpecificWorker::BottlePose compute_bottle_orientation(const RoboCompImageSegmentation::PointCloud& point_cloud);

	SpecificWorker::EulerAngles quaternion_to_euler(float qx, float qy, float qz, float qw);

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	bool simulated = configLoader.get<bool>("Simulated");
	bool display_point_cloud = configLoader.get<bool>("DisplayPointCloud");

	QWidget *viewer_widget;

	Viewer *viewer_3d;
	std::shared_ptr<std::vector<point3f>> points, colors;

	std::chrono::steady_clock::time_point bottle_dimension_display_timer;
	std::unique_ptr<DSR::RT_API> rt;

	const short Y_range_max = 350; // Maximum Y range to consider for bottle detection(forward)
	const short X_range_min = -200; // Minimum X range to consider for bottle detection(left)
	const short X_range_max = 200; // Maximum X range to consider for bottle(right)
	const short Z_range_min = -170; // Minimum Z range to consider for bottle detection(height)

	int bottle_lost_count = 0;
	int bottle_redetected_count = 0;
	const int bottle_lost_threshold = 5;
	const int bottle_redetected_threshold = 5;
	const float tilt_threshold = 45.0f; // Degrees

signals:
	//void customSignal();
};

#endif
