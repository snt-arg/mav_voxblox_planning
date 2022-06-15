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

#include <ros/wall_timer_options.h>

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
  void updateEsdf(const ros::WallTimerEvent& event);
  // Start skeleton generation
  void generateSkeleton(const ros::WallTimerEvent& event);

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

  SkeletonGenerator skeleton_generator_;

  //ros params
  FloatingPoint min_separation_angle_;
  bool generate_by_layer_neighbors_;
  int num_neighbors_for_edge_;
  FloatingPoint min_gvd_distance_;
  bool update_esdf_;
  std::string input_filepath_, output_filepath_, sparse_graph_filepath_;

  ros::WallTimer esdf_update_timer_, skeleton_generator_timer_;
};

void SkeletonizerNode::init() {
  // Load a file from the params.
  nh_private_.param("input_filepath", input_filepath_, input_filepath_);
  nh_private_.param("output_filepath", output_filepath_, output_filepath_);
  nh_private_.param("sparse_graph_filepath", sparse_graph_filepath_, sparse_graph_filepath_);
  nh_private_.param("frame_id", frame_id_, frame_id_);
  update_esdf_ = false;
  nh_private_.param("update_esdf", update_esdf_, update_esdf_);

  min_separation_angle_ = skeleton_generator_.getMinSeparationAngle();
  nh_private_.param("min_separation_angle", min_separation_angle_, min_separation_angle_);
  skeleton_generator_.setMinSeparationAngle(min_separation_angle_);
  
  generate_by_layer_neighbors_ = skeleton_generator_.getGenerateByLayerNeighbors();
  nh_private_.param("generate_by_layer_neighbors", generate_by_layer_neighbors_, generate_by_layer_neighbors_);
  skeleton_generator_.setGenerateByLayerNeighbors(generate_by_layer_neighbors_);

  num_neighbors_for_edge_ = skeleton_generator_.getNumNeighborsForEdge();
  nh_private_.param("num_neighbors_for_edge", num_neighbors_for_edge_, num_neighbors_for_edge_);
  skeleton_generator_.setNumNeighborsForEdge(num_neighbors_for_edge_);

  min_gvd_distance_ = skeleton_generator_.getMinGvdDistance();
  nh_private_.param("min_gvd_distance", min_gvd_distance_, min_gvd_distance_);
  skeleton_generator_.setMinGvdDistance(min_gvd_distance_);

  esdf_update_timer_ = nh_.createWallTimer(ros::WallDuration(1.0), &SkeletonizerNode::updateEsdf, this);
  skeleton_generator_timer_ = nh_.createWallTimer(ros::WallDuration(10.0), &SkeletonizerNode::generateSkeleton, this);
  if (input_filepath_.empty()) {
    return;
  }
}
  
void SkeletonizerNode::updateEsdf(const ros::WallTimerEvent& event) {
    //esdf_server_.loadMap(input_filepath_);

    esdf_server_.disableIncrementalUpdate();
    if (update_esdf_ ||
        esdf_server_.getEsdfMapPtr()
                ->getEsdfLayerPtr()
                ->getNumberOfAllocatedBlocks() == 0) {
      const bool full_euclidean_distance = true;
      esdf_server_.updateEsdfBatch(full_euclidean_distance);
    }

    // Visualize all parts.
    esdf_server_.updateMesh();
    esdf_server_.publishPointclouds();
    esdf_server_.publishMap();
    ROS_INFO("Finished updating ESDF.");
}

void SkeletonizerNode::generateSkeleton(const ros::WallTimerEvent& event) {
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

  // Optionally save back to file.
  // if (!output_filepath_.empty()) {
  //   // Put the TSDF, ESDF, and skeleton layer in the same bucket.
  //   if (esdf_server_.saveMap(output_filepath_)) {
  //     constexpr bool kClearFile = false;
  //     io::SaveLayer<SkeletonVoxel>(*skeleton_generator_.getSkeletonLayer(),
  //                                  output_filepath_, kClearFile);
  //     ROS_INFO("Output map to: %s", output_filepath_.c_str());
  //   } else {
  //     ROS_ERROR("Couldn't output map to: %s", output_filepath_.c_str());
  //   }
  // }
  // if (!sparse_graph_filepath_.empty()) {
  //   if (skeleton_generator_.saveSparseGraphToFile(sparse_graph_filepath_)) {
  //     ROS_INFO("Output sparse graph to: %s", sparse_graph_filepath_.c_str());
  //   } else {
  //    ROS_ERROR("Couldn't output sparse graph to: %s",
  //              sparse_graph_filepath_.c_str());
  //   }
  // }
} 

void SkeletonizerNode::skeletonize(Layer<EsdfVoxel>* esdf_layer,
                                   voxblox::Pointcloud* pointcloud,
                                   std::vector<float>* distances) {
  skeleton_generator_.setEsdfLayer(esdf_layer);
  skeleton_generator_.generateSkeleton();
  skeleton_generator_.getSkeleton().getEdgePointcloudWithDistances(pointcloud, distances);
  ROS_INFO("Finished generating skeleton.");

  skeleton_generator_.generateSparseGraph();
  ROS_INFO("Finished generating sparse graph.");
  ROS_INFO_STREAM("Total Timings: " << std::endl << timing::Timing::Print());

  // Now visualize the graph.
  const SparseSkeletonGraph& graph = skeleton_generator_.getSparseGraph();
  visualization_msgs::MarkerArray marker_array;
  visualizeSkeletonGraph(graph, frame_id_, &marker_array);
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
  //ros::Rate loop_rate(1); 
  //TODO: Add two parallel loop here
  // while (ros::ok()) {
  //   node.updateEsdf();
  //   ros::spinOnce();
  //   //loop_rate.sleep();
  //   usleep(3e6);
  // }
 
  return 0;
}
