#include <ros/wall_timer_options.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/mesh_pcl.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>

#include "voxblox_skeleton/io/skeleton_io.h"
#include "voxblox_skeleton/ros/skeleton_vis.h"
#include "voxblox_skeleton/skeleton_generator.h"

namespace voxblox {

class SkeletonizerNode {
 public:
  SkeletonizerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        frame_id_("map"),
        esdf_server_(nh_, nh_private_) {
    skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
        "skeleton", 1, true);
    sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
        "sparse_graph", 1, true);
  }

  // Initialize the node.
  void init();
  // Update ESDF
  void updateEsdf(const ros::TimerEvent& event);
  // Start skeleton generation
  void generateSkeleton(const ros::TimerEvent& event);

  // Make a skeletor!!!
  void skeletonize(Layer<EsdfVoxel>* esdf_layer, voxblox::Pointcloud* skeleton,
                   std::vector<float>* distances);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::string frame_id_;

  ros::Publisher skeleton_pub_;
  ros::Publisher sparse_graph_pub_;

  EsdfServer esdf_server_;

  // ros params
  FloatingPoint min_separation_angle_;
  bool generate_by_layer_neighbors_;
  int num_neighbors_for_edge_;
  FloatingPoint min_gvd_distance_;
  bool update_esdf_;
  std::string input_filepath_, output_filepath_, sparse_graph_filepath_;

  ros::Timer esdf_update_timer_, skeleton_generator_timer_;
};

void SkeletonizerNode::init() {
  skeleton_generator_timer_ = nh_.createTimer(
      ros::Duration(3.0), &SkeletonizerNode::generateSkeleton, this);
}

void SkeletonizerNode::generateSkeleton(const ros::TimerEvent& event) {
  // Skeletonize????
  voxblox::Pointcloud pointcloud;
  std::vector<float> distances;
  skeletonize(esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr(), &pointcloud,
              &distances);

  // Publish the skeleton.
  pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
  pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
  ptcloud_pcl.header.frame_id = frame_id_;
  skeleton_pub_.publish(ptcloud_pcl);
}

void SkeletonizerNode::skeletonize(Layer<EsdfVoxel>* esdf_layer,
                                   voxblox::Pointcloud* pointcloud,
                                   std::vector<float>* distances) {
  SkeletonGenerator skeleton_generator;
  // Load a file from the params.
  nh_private_.param("input_filepath", input_filepath_, input_filepath_);
  nh_private_.param("output_filepath", output_filepath_, output_filepath_);
  nh_private_.param("sparse_graph_filepath", sparse_graph_filepath_,
                    sparse_graph_filepath_);
  nh_private_.param("frame_id", frame_id_, frame_id_);
  update_esdf_ = false;
  nh_private_.param("update_esdf", update_esdf_, update_esdf_);

  min_separation_angle_ = skeleton_generator.getMinSeparationAngle();
  nh_private_.param("min_separation_angle", min_separation_angle_,
                    min_separation_angle_);
  skeleton_generator.setMinSeparationAngle(min_separation_angle_);

  generate_by_layer_neighbors_ =
      skeleton_generator.getGenerateByLayerNeighbors();
  nh_private_.param("generate_by_layer_neighbors", generate_by_layer_neighbors_,
                    generate_by_layer_neighbors_);
  skeleton_generator.setGenerateByLayerNeighbors(generate_by_layer_neighbors_);

  num_neighbors_for_edge_ = skeleton_generator.getNumNeighborsForEdge();
  nh_private_.param("num_neighbors_for_edge", num_neighbors_for_edge_,
                    num_neighbors_for_edge_);
  skeleton_generator.setNumNeighborsForEdge(num_neighbors_for_edge_);

  min_gvd_distance_ = skeleton_generator.getMinGvdDistance();
  nh_private_.param("min_gvd_distance", min_gvd_distance_, min_gvd_distance_);
  skeleton_generator.setMinGvdDistance(min_gvd_distance_);

  skeleton_generator.setEsdfLayer(esdf_layer);
  skeleton_generator.generateSkeleton();
  skeleton_generator.getSkeleton().getEdgePointcloudWithDistances(pointcloud,
                                                                  distances);
  ROS_INFO("Finished generating skeleton.");

  skeleton_generator.generateSparseGraph();
  ROS_INFO("Finished generating sparse graph.");
  ROS_INFO_STREAM("Total Timings: " << std::endl << timing::Timing::Print());

  // Now visualize the graph.
  const SparseSkeletonGraph& graph = skeleton_generator.getSparseGraph();
  visualization_msgs::MarkerArray marker_array;
  visualizeSkeletonGraph(graph, "map_elevated", &marker_array);
  sparse_graph_pub_.publish(marker_array);
}

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_skeletonizer");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  voxblox::SkeletonizerNode node(nh, nh_private);
  node.init();
  ros::spin();

  return 0;
}
