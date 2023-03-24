#pragma once

#include <ros/ros.h>

#include "voxblox_skeleton/skeleton_generator.h"
#include <voxblox_ros/esdf_server.h>

namespace voxblox {

class SparseSkeletonGraph;

class SkeletonServer {
public:
  SkeletonServer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  void generate();

  void publishData() const;
  void publishVisuals() const;

  const SkeletonGenerator &getSkeletonGenerator() const;
  const EsdfServer &getEsdfServer() const;

  const SparseSkeletonGraph &getSparseGraph() const;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher sparse_graph_pub_;
  ros::Publisher skeleton_layer_pub_;
  ros::Publisher skeleton_pc_vis_pub_;
  ros::Publisher sparse_graph_vis_pub_;

  ros::Subscriber sparse_graph_sub_;
  ros::Subscriber skeleton_layer_sub_;

  ros::Timer skeleton_generator_timer_;

  // ros params
  std::string frame_id_;
  bool publish_data_ = false;
  bool vis_data_ = false;
  float min_separation_angle_;
  bool generate_by_layer_neighbors_;
  int num_neighbors_for_edge_;
  float min_gvd_distance_;
  double generation_interval_ = 0.0; // disabled
  std::string input_filepath_;
  std::string output_filepath_;
  std::string sparse_graph_filepath_;

  // voxblox server
  EsdfServer esdf_server_;

  // generator
  SkeletonGenerator skeleton_generator_;
};

} // namespace voxblox