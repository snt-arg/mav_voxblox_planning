#ifndef VOXBLOX_SKELETON_ROS_SKELETON_VIS_H_
#define VOXBLOX_SKELETON_ROS_SKELETON_VIS_H_

#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/core/color.h>
#include <voxblox/core/common.h>
#include <voxblox_ros/conversions.h>

#include "voxblox_skeleton/skeleton.h"

namespace voxblox {

struct connected_vertices_struct {
public:
  int64_t id;
  bool visited;
  SkeletonVertex vertex;
  int64_t subgraph_id;
};

inline void connected_components(
    int64_t subgraph_id, const SparseSkeletonGraph &graph,
    connected_vertices_struct &connected_vertex,
    std::vector<connected_vertices_struct> &connected_vertices_struct_vec,
    std::vector<int64_t> closed_space_edge_ids);

void visualizeSkeletonGraph(const SparseSkeletonGraph &graph,
                            const std::string &frame_id,
                            visualization_msgs::MarkerArray *marker_array,
                            bool visualize_subgraphs = true,
                            bool visualize_freespace = false);

} // namespace voxblox

#endif // VOXBLOX_SKELETON_ROS_SKELETON_VIS_H_
