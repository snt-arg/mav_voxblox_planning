#include "voxblox_skeleton/skeleton_server.h"

#include "voxblox_skeleton/ros/skeleton_vis.h"

#include <voxblox_ros/conversions.h>
#include <voxblox_ros/ptcloud_vis.h>

namespace voxblox {

SkeletonServer::SkeletonServer(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), esdf_server_(nh, nh_private) {

  // params
  nh_private_.param("skeleton_publish_data", publish_data_, publish_data_);
  nh_private_.param("skeleton_visualize_data", vis_data_, vis_data_);
  nh_private_.param("skeleton_sparse_graph_filepath", sparse_graph_filepath_,
                    sparse_graph_filepath_);
  nh_private_.param("frame_id", frame_id_, frame_id_);
  nh_private_.param("skeleton_generation_interval", generation_interval_,
                    generation_interval_);

  SkeletonGenerator skeleton_generator;
  min_separation_angle_ = skeleton_generator.getMinSeparationAngle();
  nh_private_.param("skeleton_min_separation_angle", min_separation_angle_,
                    min_separation_angle_);

  generate_by_layer_neighbors_ =
      skeleton_generator.getGenerateByLayerNeighbors();
  nh_private_.param("skeleton_generate_by_layer_neighbors",
                    generate_by_layer_neighbors_, generate_by_layer_neighbors_);

  num_neighbors_for_edge_ = skeleton_generator.getNumNeighborsForEdge();
  nh_private_.param("skeleton_num_neighbors_for_edge", num_neighbors_for_edge_,
                    num_neighbors_for_edge_);

  min_gvd_distance_ = skeleton_generator.getMinGvdDistance();
  nh_private_.param("skeleton_min_gvd_distance", min_gvd_distance_,
                    min_gvd_distance_);

  // publishers
  sparse_graph_pub_ =
      nh_private_.advertise<voxblox_planning_msgs::SkeletonGraph>(
          "skeleton_sparse_graph_out", 1, true);

  skeleton_layer_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("skeleton_layer_out", 1, true);

  skeleton_pc_vis_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
      "skeleton", 1, true);

  sparse_graph_vis_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "skeleton_sparse_graph", 1, true);

  // subscribers
  sparse_graph_sub_ =
      nh_private_.subscribe<voxblox_planning_msgs::SkeletonGraph>(
          "skeleton_sparse_graph_in", 1,
          [&](const voxblox_planning_msgs::SkeletonGraph::ConstPtr &msg) {
            ROS_INFO_ONCE("Got a sparse graph from ROS topic!");
            skeleton_generator_.getSparseGraph().setFromGraphMsg(msg);
          });

  skeleton_layer_sub_ = nh_private_.subscribe<voxblox_msgs::Layer>(
      "skeleton_layer_in", 1, [&](const voxblox_msgs::Layer::ConstPtr &msg) {
        // set esdf layer which also initializes the skeleton layer
        skeleton_generator_.setEsdfLayer(
            esdf_server_.getEsdfMapPtr()
                ->getEsdfLayerPtr()); // also initializes the skeleton layer

        ROS_INFO_ONCE("Got a skeleton layer from ROS topic!");

        // note: we could also just update the map here
        deserializeMsgToLayer(*msg, MapDerializationAction::kUpdate,
                              skeleton_generator_.getSkeletonLayer());
      });

  // timers
  if (generation_interval_ > 0.0) {
    skeleton_generator_timer_ =
        nh_.createTimer(ros::Duration(generation_interval_),
                        [&](const ros::TimerEvent &) { generate(); });
  } else {
    ROS_INFO("Skeleton generation disabled");
  }
}

void SkeletonServer::generate() {
  // start with a fresh skeleton
  skeleton_generator_ = SkeletonGenerator();

  // apply user params
  skeleton_generator_.setMinSeparationAngle(min_separation_angle_);
  skeleton_generator_.setGenerateByLayerNeighbors(generate_by_layer_neighbors_);
  skeleton_generator_.setNumNeighborsForEdge(num_neighbors_for_edge_);
  skeleton_generator_.setMinGvdDistance(min_gvd_distance_);

  // set esdf layer
  skeleton_generator_.setEsdfLayer(
      esdf_server_.getEsdfMapPtr()
          ->getEsdfLayerPtr()); // also initializes the skeleton layer

  // generate skeleton
  skeleton_generator_.generateSkeleton();

  // generate sparse graph
  skeleton_generator_.generateSparseGraph();

  // pub
  if (publish_data_) {
    publishData();
  }
  if (vis_data_) {
    publishVisuals();
  }
}

const SkeletonGenerator &SkeletonServer::getSkeletonGenerator() const {
  return skeleton_generator_;
}

const EsdfServer &SkeletonServer::getEsdfServer() const { return esdf_server_; }

void SkeletonServer::publishData() const {

  sparse_graph_pub_.publish(skeleton_generator_.getSparseGraph().toGraphMsg());

  if (skeleton_generator_.getSkeletonLayer()) {
    voxblox_msgs::Layer layer_msg;
    serializeLayerAsMsg<SkeletonVoxel>(*skeleton_generator_.getSkeletonLayer(),
                                       false, &layer_msg);
    skeleton_layer_pub_.publish(layer_msg);
  }
}

void SkeletonServer::publishVisuals() const {
  // skeleton pointcloud
  Pointcloud pointcloud;
  std::vector<float> distances;

  skeleton_generator_.getSkeleton().getEdgePointcloudWithDistances(&pointcloud,
                                                                   &distances);

  pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
  pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
  ptcloud_pcl.header.frame_id = frame_id_;
  skeleton_pc_vis_pub_.publish(ptcloud_pcl);

  // sparse graph markers
  const SparseSkeletonGraph &graph = skeleton_generator_.getSparseGraph();
  visualization_msgs::MarkerArray marker_array;
  visualizeSkeletonGraph(graph, "map", &marker_array);
  sparse_graph_vis_pub_.publish(marker_array);
}

const SparseSkeletonGraph &SkeletonServer::getSparseGraph() const {
  return skeleton_generator_.getSparseGraph();
}

} // namespace voxblox