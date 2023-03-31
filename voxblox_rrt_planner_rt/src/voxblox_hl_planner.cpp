#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/utils/planning_utils.h>

#include "voxblox_msgs/Layer.h"
#include "voxblox_rrt_planner/voxblox_hl_planner.h"

namespace mav_planning {

/*
  roslaunch stugalux_spot_bringup system.launch env:=virtual
  operation_mode:=auto_ using_free_space_graph:=true playing_rosbag:=true rosbag
  play building_6rooms_4r.bag --clock rviz -d s_graphs.rviz

  roslaunch voxblox_skeleton skeleton_server.launch
  roslaunch voxblox_rrt_planner_rt sparse_planner.launch

*/

VoxbloxHlPlanner::VoxbloxHlPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh)
    , nh_private_(nh_private)
    , frame_id_("odom")
    , visualize_(true)
    , do_smoothing_(true)
    , last_trajectory_valid_(true)
    , lower_bound_(Eigen::Vector3d::Zero())
    , upper_bound_(Eigen::Vector3d::Zero())
    , skeleton_server_(nh, nh_private)
    , skeleton_graph_planner_(nh_, nh_private_)
    , rrt_(nh_, nh_private_)
{
    constraints_.setParametersFromRos(nh_private_);

    std::string input_filepath;
    nh_private_.param("voxblox_path", input_filepath, input_filepath);
    nh_private_.param("visualize", visualize_, visualize_);
    nh_private_.param("frame_id", frame_id_, frame_id_);
    nh_private_.param("do_smoothing", do_smoothing_, do_smoothing_);

    // publishers
    path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true); // raw path in the sparse graph
    skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ>>("skeleton", 1, true);
    sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("sparse_graph", 1, true);

    polynomial_trajectory_pub_ = nh_private_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("polynomial_trajectory", 1);
    trajectory_vis_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("trajectory_vis", 1);
    waypoint_list_pub_ = nh_private_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);
    waypoint_list_2d_pub_ = nh_private_.advertise<geometry_msgs::PoseArray>("waypoint_list_2d", 1);
    waypoint_list_2d_vis_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("waypoint_list_2d_vis", 1);
    exploration_candidates_vis_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("exploration_candidates_vis", 1);
    exploration_frontier_vis_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("exploration_frontier_layer_vis", 1);

    planner_srv_ = nh_private_.advertiseService("plan", &VoxbloxHlPlanner::plannerServiceCallback, this);
    path_pub_srv_ = nh_private_.advertiseService("publish_path", &VoxbloxHlPlanner::publishPathCallback, this);

    esdf_map_ = skeleton_server_.getEsdfServer().getEsdfMapPtr();
    tsdf_map_ = skeleton_server_.getEsdfServer().getTsdfMapPtr();

    const double voxel_size = esdf_map_->getEsdfLayerPtr()->voxel_size();

    skeleton_graph_planner_.setRobotRadius(constraints_.robot_radius);
    skeleton_graph_planner_.setEsdfLayer(skeleton_server_.getEsdfServer().getEsdfMapPtr()->getEsdfLayerPtr());

    // Set up the path smoother as well.
    smoother_.setParametersFromRos(nh_private_);
    smoother_.setMinCollisionCheckResolution(voxel_size);
    smoother_.setMapDistanceCallback(std::bind(&VoxbloxHlPlanner::getMapDistance, this, std::placeholders::_1));

    // Loco smoother!
    loco_smoother_.setParametersFromRos(nh_private_);
    loco_smoother_.setMinCollisionCheckResolution(voxel_size);
    loco_smoother_.setMapDistanceCallback(std::bind(&VoxbloxHlPlanner::getMapDistance, this, std::placeholders::_1));

    // rrt
    ROS_INFO("Robot radius %f", constraints_.robot_radius);
    rrt_.setRobotRadius(constraints_.robot_radius);
    rrt_.setOptimistic(false);

    // exploration candidate generation
    explr_timer_ = nh_private.createTimer(ros::Duration(1), [&](const ros::TimerEvent&) {
        // findExplorationCandidates();
        findExplorationBoundary();
        findSparseExplorationCandidates();
    });
}

bool VoxbloxHlPlanner::publishPathCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response)
{
    if (!last_trajectory_valid_) {
        ROS_ERROR("Can't publish trajectory, marked as invalid.");
        return false;
    }

    ROS_INFO("Publishing path.");

    if (!do_smoothing_) {
        {
            geometry_msgs::PoseArray pose_array;
            pose_array.poses.reserve(last_graph_path_.size());
            for (const mav_msgs::EigenTrajectoryPoint& point : last_graph_path_) {
                geometry_msgs::PoseStamped pose_stamped;
                mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(point, &pose_stamped);
                pose_array.poses.push_back(pose_stamped.pose);
            }

            pose_array.header.frame_id = frame_id_;
            waypoint_list_pub_.publish(pose_array);
        }

        {
            // flatten
            auto graph_path_2d = last_graph_path_;
            for (auto& p : graph_path_2d) {
                p.position_W[2] = 0.5;
            }

            // geometry_msgs::PoseArray pose_array;
            // pose_array.poses.reserve(last_graph_path_.size());
            // for (const mav_msgs::EigenTrajectoryPoint& point : last_graph_path_) {
            //     geometry_msgs::PoseStamped pose_stamped;
            //     mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(point, &pose_stamped);
            //     pose_stamped.pose.position.z = 0.0;
            //     pose_array.poses.push_back(pose_stamped.pose);
            // }
            // pose_array.header.frame_id = frame_id_;

            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.push_back(createMarkerForPath(graph_path_2d, frame_id_, mav_visualization::Color::Red(), "graph_plan_2d", 0.1));
            waypoint_list_2d_vis_pub_.publish(marker_array);
        }

        ROS_INFO("Published %i", last_waypoints_.size());
    } else {
        // mav_trajectory_generation::Trajectory trajectory;
        mav_msgs::EigenTrajectoryPointVector trajectory;

        // smoother_.setSplitAtCollisions(true);
        // bool success = smoother_.getTrajectoryBetweenWaypoints(last_graph_path_, &trajectory);

        bool success = generateFeasibleTrajectoryLoco(last_graph_path_, &trajectory);

        // loco_smoother_.setNumSegments(5);
        // bool success = loco_smoother_.getTrajectoryBetweenWaypoints(last_graph_path_, &trajectory);

        if (success) {

            // mav_planning_msgs::PolynomialTrajectory4D msg;
            // mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);

            // msg.header.stamp = ros::Time::now();
            // msg.header.frame_id = frame_id_;
            // polynomial_trajectory_pub_.publish(msg);

            // mav_trajectory_generation::Trajectory trajectory;
            // mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(msg, &trajectory);

            visualization_msgs::MarkerArray marker_array;
            mav_trajectory_generation::drawMavSampledTrajectory(trajectory, 1.0, frame_id_, &marker_array);
            // mav_trajectory_generation::drawMavTrajectory(trajectory, 0.0, frame_id_, &marker_array);
            trajectory_vis_pub_.publish(marker_array);

            ROS_INFO("Successfully calculated smoothened trajectory!");
        } else {
            ROS_INFO("Failed to calculate smoothened trajectory");
        }
    }

    return true;
}

void VoxbloxHlPlanner::computeMapBounds(Eigen::Vector3d* lower_bound, Eigen::Vector3d* upper_bound) const
{
    if (esdf_map_) {
        voxblox::utils::computeMapBoundsFromLayer(*esdf_map_->getEsdfLayerConstPtr(), lower_bound, upper_bound);
    } else if (tsdf_map_) {
        voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerConstPtr(), lower_bound, upper_bound);
    }
}

bool VoxbloxHlPlanner::plannerServiceCallback(mav_planning_msgs::PlannerServiceRequest& request, mav_planning_msgs::PlannerServiceResponse& response)
{
    bool success = false;
    switch (planner_type_) {
    case PlannerType::RRT:
        success = plannerServiceCallbackRRT(request, response);
        break;
    case PlannerType::SparseGraph:
        success = plannerServiceCallbackSparseGraph(request, response);
        break;
    }

    if (visualize_) {
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(createMarkerForPath(last_graph_path_, frame_id_, mav_visualization::Color::Blue(), "graph_plan", 0.1));
        path_marker_pub_.publish(marker_array);
    }

    return success;
}

bool VoxbloxHlPlanner::plannerServiceCallbackRRT(mav_planning_msgs::PlannerServiceRequest& request, mav_planning_msgs::PlannerServiceResponse& response)
{
    mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;

    mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
    mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);

    auto esdf_map = skeleton_server_.getEsdfServer().getEsdfMapPtr();
    auto tsdf_map = skeleton_server_.getEsdfServer().getTsdfMapPtr();

    // Setup latest copy of map.
    if (!(esdf_map_ && esdf_map_->getEsdfLayerConstPtr()->getNumberOfAllocatedBlocks() > 0)
        && !(tsdf_map_ && tsdf_map_->getTsdfLayerConstPtr()->getNumberOfAllocatedBlocks() > 0)) {
        ROS_ERROR("Both maps are empty!");
        return false;
    }
    rrt_.setTsdfLayer(tsdf_map->getTsdfLayerConstPtr());
    rrt_.setEsdfLayer(esdf_map->getEsdfLayerConstPtr());

    // Figure out map bounds!
    computeMapBounds(&lower_bound_, &upper_bound_);

    ROS_INFO_STREAM("Map bounds: " << lower_bound_.transpose() << " to " << upper_bound_.transpose() << " size: " << (upper_bound_ - lower_bound_).transpose());

    // Inflate the bounds a bit.
    constexpr double kBoundInflationMeters = 0.5;
    // Don't in flate in z. ;)
    ROS_INFO("Set bounds.");
    auto center = (upper_bound_[2] + lower_bound_[2]) / 2.0;
    upper_bound_[2] = 0.8; // center + 0.1;
    lower_bound_[2] = 0.2; // center - 0.1;
    start_pose.position_W[2] = (upper_bound_[2] + lower_bound_[2]) / 2.0;
    goal_pose.position_W[2] = (upper_bound_[2] + lower_bound_[2]) / 2.0;
    ROS_INFO("Z bounds %f, %f, %f", center, lower_bound_[2], upper_bound_[2]);
    rrt_.setBounds(lower_bound_ - Eigen::Vector3d(kBoundInflationMeters, kBoundInflationMeters, 0.0),
        upper_bound_ + Eigen::Vector3d(kBoundInflationMeters, kBoundInflationMeters, 0.0));
    ROS_INFO("Setup problem.");
    rrt_.setupProblem();

    ROS_INFO("Planning path.");

    if (getMapDistance(start_pose.position_W) < constraints_.robot_radius) {
        ROS_ERROR("Start pose occupied!");
        return false;
    }
    if (getMapDistance(goal_pose.position_W) < constraints_.robot_radius) {
        ROS_ERROR("Goal pose occupied!");
        return false;
    }

    mav_msgs::EigenTrajectoryPoint::Vector waypoints;
    mav_trajectory_generation::timing::Timer rrtstar_timer("plan/rrt_star");

    bool success = rrt_.getPathBetweenWaypoints(start_pose, goal_pose, &waypoints);
    rrtstar_timer.Stop();
    double path_length = computePathLength(waypoints);
    int num_vertices = waypoints.size();
    ROS_INFO("RRT* Success? %d Path length: %f Vertices: %d", success, path_length, num_vertices);

    if (!success) {
        return false;
    }

    visualization_msgs::MarkerArray marker_array;
    if (visualize_) {
        marker_array.markers.push_back(createMarkerForPath(waypoints, frame_id_, mav_visualization::Color::Green(), "rrt_star", 0.075));
        marker_array.markers.push_back(createMarkerForWaypoints(waypoints, frame_id_, mav_visualization::Color::Green(), "rrt_star_waypoints", 0.15));
    }

    last_waypoints_ = waypoints;
    last_graph_path_ = waypoints;

    if (!do_smoothing_) {
        last_trajectory_valid_ = true;
    } else {
        mav_msgs::EigenTrajectoryPointVector poly_path;
        mav_trajectory_generation::timing::Timer poly_timer("plan/poly");
        bool poly_has_collisions = !generateFeasibleTrajectory(waypoints, &poly_path);
        poly_timer.Stop();

        mav_msgs::EigenTrajectoryPointVector loco_path;
        mav_trajectory_generation::timing::Timer loco_timer("plan/loco");
        bool loco_has_collisions = !generateFeasibleTrajectoryLoco(waypoints, &loco_path);
        loco_timer.Stop();

        mav_msgs::EigenTrajectoryPointVector loco2_path;
        mav_trajectory_generation::timing::Timer loco2_timer("plan/loco2");
        bool loco2_has_collisions = !generateFeasibleTrajectoryLoco2(waypoints, &loco2_path);
        loco2_timer.Stop();

        ROS_INFO("Poly Smoothed Path has collisions? %d Loco Path has collisions? %d "
                 "Loco 2 has collisions? %d",
            poly_has_collisions, loco_has_collisions, loco2_has_collisions);

        if (!poly_has_collisions) {
            last_trajectory_valid_ = true;
        }
    }

    response.success = success;

    ROS_INFO_STREAM("All timings: " << std::endl << mav_trajectory_generation::timing::Timing::Print());
    ROS_INFO_STREAM("Finished planning with start point: " << start_pose.position_W.transpose() << " and goal point: " << goal_pose.position_W.transpose());
    return success;
}

bool VoxbloxHlPlanner::plannerServiceCallbackSparseGraph(mav_planning_msgs::PlannerServiceRequest& request, mav_planning_msgs::PlannerServiceResponse& response)
{
    if (skeleton_server_.getSkeletonGenerator().getSkeletonLayer()) {
        skeleton_graph_planner_.setSkeletonLayer(skeleton_server_.getSkeletonGenerator().getSkeletonLayer());
    } else {
        ROS_ERROR("Skeleton not generated!");
        return false;
    }

    mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;

    mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
    mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);
    mav_msgs::EigenTrajectoryPointVector graph_path;

    ROS_INFO_NAMED("SparsePlanner", "getPathBetweenWaypoints");
    mav_trajectory_generation::timing::Timer graph_timer("plan/graph");
    skeleton_graph_planner_.setShortenPath(true); // checks ESDF map
    skeleton_graph_planner_.setSparseGraph(&skeleton_server_.getSkeletonGenerator().getSparseGraph());
    bool success = skeleton_graph_planner_.getPathBetweenWaypoints(start_pose, goal_pose, &graph_path);

    ROS_INFO_NAMED("SparsePlanner", "getPathBetweenWaypoints done");
    double path_length = computePathLength(graph_path);
    int num_vertices = graph_path.size();
    graph_timer.Stop();
    ROS_INFO_NAMED("SparsePlanner", "Graph Planning Success? %d Path length: %f Vertices: %d", success, path_length, num_vertices);

    if (visualize_) {
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(createMarkerForPath(graph_path, frame_id_, mav_visualization::Color::Blue(), "graph_plan", 0.1));
        path_marker_pub_.publish(marker_array);
    }

    last_graph_path_ = graph_path;

    return success;
}

bool VoxbloxHlPlanner::generateFeasibleTrajectory(const mav_msgs::EigenTrajectoryPointVector& coordinate_path, mav_msgs::EigenTrajectoryPointVector* path)
{
    smoother_.getPathBetweenWaypoints(coordinate_path, path);

    bool path_in_collision = checkPathForCollisions(*path, NULL);

    if (path_in_collision) {
        return false;
    }
    return true;
}

bool VoxbloxHlPlanner::generateFeasibleTrajectoryLoco(const mav_msgs::EigenTrajectoryPointVector& coordinate_path, mav_msgs::EigenTrajectoryPointVector* path)
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

bool VoxbloxHlPlanner::generateFeasibleTrajectoryLoco2(const mav_msgs::EigenTrajectoryPointVector& coordinate_path, mav_msgs::EigenTrajectoryPointVector* path)
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

bool VoxbloxHlPlanner::checkPathForCollisions(const mav_msgs::EigenTrajectoryPointVector& path, double* t) const
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

double VoxbloxHlPlanner::getMapDistance(const Eigen::Vector3d& position) const
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

bool VoxbloxHlPlanner::checkPhysicalConstraints(const mav_trajectory_generation::Trajectory& trajectory)
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

// \brief
// find vertices worth exploring by checking which vertices have no allocated blocks nearby
void VoxbloxHlPlanner::findSparseExplorationCandidates()
{
    if (!frontier_layer_) {
        return;
    }

    float slice_z = 0.5;
    const auto& tsdf_layer = skeleton_server_.getEsdfServer().getTsdfMapPtr()->getTsdfLayer();
    const float voxel_size = tsdf_layer.voxel_size();

    if (std::remainder(slice_z, voxel_size) < 1e-4) {
        slice_z += voxel_size / 2.0;
    }

    ROS_INFO("Find explration candidates");

    const float radius = 0.8;

    std::vector<int64_t> sg_vertices;
    skeleton_server_.getSparseGraph().getAllVertexIds(&sg_vertices);

    std::vector<int64_t> sg_candidate_vertices;

    std::vector<int> freespace_candidates;
    for (auto index : sg_vertices) {
        const auto& vertex_data = skeleton_server_.getSparseGraph().getVertex(index);
        auto pos = vertex_data.point;
        pos.z() = slice_z;

        const int steps = std::ceil(radius / voxel_size);
        const int steps_theta = 16;

        bool good_candidate = false;

        const auto this_voxel = frontier_layer_->getVoxelPtrByCoordinates(pos);

        int gc = 0;

        // circle
        for (int theta_n = 1; theta_n <= steps_theta; ++theta_n) {
            const float theta = (2 * M_PI / steps_theta) * theta_n;
            const voxblox::Point dir = voxblox::Point(std::cos(theta), std::sin(theta), 0);

            // line
            for (int i = 1; i < steps; ++i) {

                const auto tsdf_voxel = tsdf_layer.getVoxelPtrByCoordinates(pos + dir * i * voxel_size);
                const auto frontier_voxel = frontier_layer_->getVoxelPtrByCoordinates(pos + dir * i * voxel_size);

                if (!tsdf_voxel || (tsdf_voxel && tsdf_voxel->distance < 0 && tsdf_voxel->weight > 1e-3)) {
                    break;
                }

                if (frontier_voxel && frontier_voxel->probability_log > 0.6 && frontier_voxel->observed) {
                    gc++;
                }
            }
        }

        if (gc > 2) {
            sg_candidate_vertices.push_back(index);
        }
    }

    ROS_INFO("Found %i candidates to explore", sg_candidate_vertices.size());

    // visualize
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.ns = "explr_candidates";
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    marker.scale.x = voxel_size;
    marker.scale.y = voxel_size;
    marker.scale.z = voxel_size;
    marker.header.frame_id = frame_id_;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;

    for (auto index : sg_candidate_vertices) {
        const auto& vertex_data = skeleton_server_.getSparseGraph().getVertex(index);
        geometry_msgs::Point point;
        point.x = vertex_data.point.x();
        point.y = vertex_data.point.y();
        point.z = vertex_data.point.z();
        marker.points.push_back(point);
    }

    markers.markers.push_back(marker);

    exploration_candidates_vis_pub_.publish(markers);
}

void VoxbloxHlPlanner::findExplorationBoundary()
{
    using namespace voxblox;

    if (!skeleton_server_.getEsdfServer().getTsdfMapPtr()) {
        return;
    }

    float slice_z = 0.5;
    const auto& tsdf = skeleton_server_.getEsdfServer().getTsdfMapPtr()->getTsdfLayer();
    const float voxel_size = tsdf.voxel_size();

    if (std::remainder(slice_z, voxel_size) < 1e-4) {
        slice_z += voxel_size / 2.0;
    }

    std::vector<voxblox::Point> candidate_points;

    voxblox::BlockIndexList block_indices;
    tsdf.getAllAllocatedBlocks(&block_indices);

    const size_t vps = tsdf.voxels_per_side();
    const size_t num_voxels_per_block = vps * vps * vps;

    // TODO: update rather then recreate
    // create a new voxblox layer with frontier voxels
    // if (!frontier_layer) {
    frontier_layer_ = std::make_shared<Layer<OccupancyVoxel>>(voxel_size, vps);
    // }

    const std::array<voxblox::VoxelIndex, 8> neighbours = {
        voxblox::VoxelIndex(1, 0, 0),
        voxblox::VoxelIndex(1, 1, 0),
        voxblox::VoxelIndex(1, -1, 0),
        voxblox::VoxelIndex(-1, 0, 0),
        voxblox::VoxelIndex(-1, 1, 0),
        voxblox::VoxelIndex(-1, -1, 0),
        voxblox::VoxelIndex(0, 1, 0),
        voxblox::VoxelIndex(0, -1, 0),
    };

    for (auto block_index : block_indices) {
        const auto& block = tsdf.getBlockByIndex(block_index);

        for (auto linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            const auto& voxel = block.getVoxelByLinearIndex(linear_index);
            const auto voxel_index = block.computeVoxelIndexFromLinearIndex(linear_index);
            const auto pos = block.computeCoordinatesFromLinearIndex(linear_index);
            auto global_index = getGlobalVoxelIndexFromBlockAndVoxelIndex(block_index, voxel_index, vps);

            if (std::abs(pos.z() - slice_z) < voxel_size / 2) {
                // if this is an explored voxel in free space
                if (voxel.weight > 1e-3 && voxel.distance > 0) {
                    bool candidate = false;
                    // check the 8 direct neighbours for unexplored territory
                    for (const auto& np : neighbours) {
                        auto nb_global_index = global_index + np.cast<long int>();
                        auto nv = tsdf.getVoxelPtrByGlobalIndex(nb_global_index);
                        if (nv) {
                            if (nv->weight == 0) {
                                candidate = true;
                                break;
                            }
                        } else {
                            candidate = true;
                            break;
                        }
                    }

                    // add candidate do the frontier layer
                    if (candidate) {
                        auto frontier_block = frontier_layer_->allocateBlockPtrByIndex(block_index);
                        auto voxel = frontier_layer_->getVoxelPtrByGlobalIndex(global_index);
                        if (voxel) {
                            voxel->observed = true;
                            voxel->probability_log = 0.99;
                        }

                        candidate_points.push_back(pos);
                    }
                }
            }
        }
    }

    // sparsen
    auto new_frontier_layer = frontier_layer_;
    frontier_layer_ = std::make_shared<Layer<OccupancyVoxel>>(voxel_size, vps);

    new_frontier_layer->getAllAllocatedBlocks(&block_indices);
    for (auto block_index : block_indices) {
        auto& block = new_frontier_layer->getBlockByIndex(block_index);

        for (auto linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            auto& voxel = block.getVoxelByLinearIndex(linear_index);
            const auto voxel_index = block.computeVoxelIndexFromLinearIndex(linear_index);
            const auto pos = block.computeCoordinatesFromLinearIndex(linear_index);
            const auto global_index = getGlobalVoxelIndexFromBlockAndVoxelIndex(block_index, voxel_index, vps);

            if (!voxel.observed) {
                continue;
            }

            int nb_count = 0;
            for (const auto& nbp : neighbours) {
                auto nb_global_index = global_index + nbp.cast<long int>();
                auto nb_voxel = new_frontier_layer->getVoxelPtrByGlobalIndex(nb_global_index);

                if (nb_voxel && nb_voxel->observed) {
                    nb_count++;
                }
            }

            if (nb_count > 0) {
                auto block = frontier_layer_->allocateBlockPtrByIndex(block_index);
                auto voxel = frontier_layer_->getVoxelPtrByGlobalIndex(global_index);
                if (voxel) {
                    voxel->observed = true;
                    voxel->probability_log = 0.99;
                }
            }
        }
    }

    // use the default voxblox visualization
    visualization_msgs::MarkerArray markers;
    voxblox::createOccupancyBlocksFromOccupancyLayer(*frontier_layer_, frame_id_, &markers);
    exploration_frontier_vis_pub_.publish(markers);
}

} // namespace mav_planning
