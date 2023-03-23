#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <voxblox/utils/planning_utils.h>

#include "voxblox_msgs/Layer.h"
#include "voxblox_rrt_planner/voxblox_sparse_planner.h"

namespace mav_planning {

/*
  roslaunch stugalux_spot_bringup system.launch env:=virtual
  operation_mode:=auto_ using_free_space_graph:=true playing_rosbag:=true rosbag
  play building_6rooms_4r.bag --clock rviz -d s_graphs.rviz

  roslaunch voxblox_skeleton skeleton_server.launch
  roslaunch voxblox_rrt_planner_rt sparse_planner.launch

*/

VoxbloxSparsePlanner::VoxbloxSparsePlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh)
    , nh_private_(nh_private)
    , frame_id_("odom")
    , visualize_(true)
    , do_smoothing_(true)
    , last_trajectory_valid_(false)
    , lower_bound_(Eigen::Vector3d::Zero())
    , upper_bound_(Eigen::Vector3d::Zero())
    , skeleton_server_(nh, nh_private)
    , skeleton_graph_planner_(nh_, nh_private_)
{
    constraints_.setParametersFromRos(nh_private_);

    std::string input_filepath;
    nh_private_.param("voxblox_path", input_filepath, input_filepath);
    nh_private_.param("visualize", visualize_, visualize_);
    nh_private_.param("frame_id", frame_id_, frame_id_);
    nh_private_.param("do_smoothing", do_smoothing_, do_smoothing_);

    // publishers
    path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
    skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ>>("skeleton", 1, true);
    sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("sparse_graph", 1, true);

    waypoint_list_pub_ = nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);

    planner_srv_ = nh_private_.advertiseService("plan", &VoxbloxSparsePlanner::plannerServiceCallback, this);
    path_pub_srv_ = nh_private_.advertiseService("publish_path", &VoxbloxSparsePlanner::publishPathCallback, this);

    esdf_map_ = skeleton_server_.getEsdfServer().getEsdfMapPtr();
    tsdf_map_ = skeleton_server_.getEsdfServer().getTsdfMapPtr();

    double voxel_size = esdf_map_->getEsdfLayerPtr()->voxel_size();

    skeleton_graph_planner_.setRobotRadius(constraints_.robot_radius);
    skeleton_graph_planner_.setEsdfLayer(skeleton_server_.getEsdfServer().getEsdfMapPtr()->getEsdfLayerPtr());

    // Set up the path smoother as well.
    smoother_.setParametersFromRos(nh_private_);
    smoother_.setMinCollisionCheckResolution(voxel_size);
    smoother_.setMapDistanceCallback(std::bind(&VoxbloxSparsePlanner::getMapDistance, this, std::placeholders::_1));

    // Loco smoother!
    loco_smoother_.setParametersFromRos(nh_private_);
    loco_smoother_.setMinCollisionCheckResolution(voxel_size);
    loco_smoother_.setMapDistanceCallback(std::bind(&VoxbloxSparsePlanner::getMapDistance, this, std::placeholders::_1));
}

bool VoxbloxSparsePlanner::publishPathCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response)
{
    if (!last_trajectory_valid_) {
        ROS_ERROR("Can't publish trajectory, marked as invalid.");
        return false;
    }

    ROS_INFO("Publishing path.");

    if (!do_smoothing_) {
        geometry_msgs::PoseArray pose_array;
        pose_array.poses.reserve(last_waypoints_.size());
        for (const mav_msgs::EigenTrajectoryPoint& point : last_waypoints_) {
            geometry_msgs::PoseStamped pose_stamped;
            mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(point, &pose_stamped);
            pose_array.poses.push_back(pose_stamped.pose);
        }

        pose_array.header.frame_id = frame_id_;
        waypoint_list_pub_.publish(pose_array);
    } else {
        mav_planning_msgs::PolynomialTrajectory4D msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(last_trajectory_, &msg);

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id_;
        polynomial_trajectory_pub_.publish(msg);
    }
    return true;
}

void VoxbloxSparsePlanner::computeMapBounds(Eigen::Vector3d* lower_bound, Eigen::Vector3d* upper_bound) const
{
    if (esdf_map_) {
        voxblox::utils::computeMapBoundsFromLayer(*esdf_map_->getEsdfLayerConstPtr(), lower_bound, upper_bound);
    } else if (tsdf_map_) {
        voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerConstPtr(), lower_bound, upper_bound);
    }
}

bool VoxbloxSparsePlanner::plannerServiceCallback(mav_planning_msgs::PlannerServiceRequest& request, mav_planning_msgs::PlannerServiceResponse& response)
{

    if (skeleton_server_.getSkeletonGenerator().getSkeletonLayer()) {
        skeleton_graph_planner_.setSkeletonLayer(skeleton_server_.getSkeletonGenerator().getSkeletonLayer());
    } else {
        ROS_ERROR("Skeleton not generated!");
        return false;
    }

    mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;
    visualization_msgs::MarkerArray marker_array;

    mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
    mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);
    mav_msgs::EigenTrajectoryPointVector graph_path;

    ROS_INFO_NAMED("SparsePlanner", "getPathBetweenWaypoints");
    mav_trajectory_generation::timing::Timer graph_timer("plan/graph");
    skeleton_graph_planner_.setShortenPath(false);
    skeleton_graph_planner_.setSparseGraph(&skeleton_server_.getSkeletonGenerator().getSparseGraph());
    bool success = skeleton_graph_planner_.getPathBetweenWaypoints(start_pose, goal_pose, &graph_path);

    ROS_INFO_NAMED("SparsePlanner", "getPathBetweenWaypoints done");
    double path_length = computePathLength(graph_path);
    int num_vertices = graph_path.size();
    graph_timer.Stop();
    ROS_INFO_NAMED("SparsePlanner", "Graph Planning Success? %d Path length: %f Vertices: %d", success, path_length, num_vertices);

    if (visualize_) {
        marker_array.markers.push_back(createMarkerForPath(graph_path, frame_id_, mav_visualization::Color::Blue(), "graph_plan", 0.1));
        path_marker_pub_.publish(marker_array);
    }

    return success;
}

bool VoxbloxSparsePlanner::generateFeasibleTrajectory(const mav_msgs::EigenTrajectoryPointVector& coordinate_path, mav_msgs::EigenTrajectoryPointVector* path)
{
    smoother_.getPathBetweenWaypoints(coordinate_path, path);

    bool path_in_collision = checkPathForCollisions(*path, NULL);

    if (path_in_collision) {
        return false;
    }
    return true;
}

bool VoxbloxSparsePlanner::generateFeasibleTrajectoryLoco(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path, mav_msgs::EigenTrajectoryPointVector* path)
{
    loco_smoother_.setResampleTrajectory(false);
    loco_smoother_.setAddWaypoints(false);

    loco_smoother_.getPathBetweenWaypoints(coordinate_path, path);

    bool path_in_collision = checkPathForCollisions(*path, NULL);

    if (path_in_collision) {
        return false;
    }
    return true;
}

bool VoxbloxSparsePlanner::generateFeasibleTrajectoryLoco2(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path, mav_msgs::EigenTrajectoryPointVector* path)
{
    loco_smoother_.setResampleTrajectory(true);
    loco_smoother_.setAddWaypoints(false);

    loco_smoother_.getPathBetweenWaypoints(coordinate_path, path);

    bool path_in_collision = checkPathForCollisions(*path, NULL);

    if (path_in_collision) {
        return false;
    }
    return true;
}

bool VoxbloxSparsePlanner::checkPathForCollisions(const mav_msgs::EigenTrajectoryPointVector& path, double* t) const
{
    for (const mav_msgs::EigenTrajectoryPoint& point : path) {
        if (getMapDistance(point.position_W) < constraints_.robot_radius) {
            if (t != NULL) {
                *t = mav_msgs::nanosecondsToSeconds(point.time_from_start_ns);
            }
            return true;
        }
    }
    return false;
}

double VoxbloxSparsePlanner::getMapDistance(const Eigen::Vector3d& position) const
{
    if (!skeleton_server_.getEsdfServer().getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!skeleton_server_.getEsdfServer().getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool VoxbloxSparsePlanner::checkPhysicalConstraints(const mav_trajectory_generation::Trajectory& trajectory)
{
    // Check min/max manually.
    // Evaluate min/max extrema
    std::vector<int> dimensions = { 0, 1, 2 }; // Evaluate dimensions in x, y and z
    mav_trajectory_generation::Extremum v_min, v_max, a_min, a_max;
    trajectory.computeMinMaxMagnitude(mav_trajectory_generation::derivative_order::VELOCITY, dimensions, &v_min, &v_max);
    trajectory.computeMinMaxMagnitude(mav_trajectory_generation::derivative_order::ACCELERATION, dimensions, &a_min, &a_max);

    ROS_INFO("V min/max: %f/%f, A min/max: %f/%f", v_min.value, v_max.value, a_min.value, a_max.value);

    // Create input constraints.
    // TODO(helenol): just store these as members...
    typedef mav_trajectory_generation::InputConstraintType ICT;
    mav_trajectory_generation::InputConstraints input_constraints;
    input_constraints.addConstraint(ICT::kFMin, (mav_msgs::kGravity - constraints_.a_max)); // maximum acceleration in [m/s/s].
    input_constraints.addConstraint(ICT::kFMax, (mav_msgs::kGravity + constraints_.a_max)); // maximum acceleration in [m/s/s].
    input_constraints.addConstraint(ICT::kVMax, constraints_.v_max); // maximum velocity in [m/s].

    // Create feasibility object of choice (FeasibilityAnalytic,
    // FeasibilitySampling, FeasibilityRecursive).
    mav_trajectory_generation::FeasibilityAnalytic feasibility_check(input_constraints);
    feasibility_check.settings_.setMinSectionTimeS(0.01);

    mav_trajectory_generation::InputFeasibilityResult feasibility = feasibility_check.checkInputFeasibilityTrajectory(trajectory);
    if (feasibility != mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
        ROS_ERROR_STREAM("Trajectory is input infeasible: " << mav_trajectory_generation::getInputFeasibilityResultName(feasibility));
        return false;
    }
    return true;
}

} // namespace mav_planning
