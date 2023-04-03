#include "voxblox_skeleton/ros/bim_vis.h"

#include <Eigen/Geometry>
#include <ros/console.h>

void visualizeIntersectionLayer(
    const voxblox::Layer<bim::IntersectionVoxel> &intersection_layer,
    visualization_msgs::MarkerArray *markers, const std::string &frame_id) {
  CHECK_NOTNULL(markers);

  voxblox::BlockIndexList blocks;
  intersection_layer.getAllAllocatedBlocks(&blocks);

  auto voxel_size = intersection_layer.voxel_size();

  visualization_msgs::Marker marker;
  marker.ns = "intersection_layer";
  marker.pose.orientation.w = 1.0;
  marker.scale.x = voxel_size;
  marker.scale.y = voxel_size;
  marker.scale.z = voxel_size;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0;
  marker.color.b = 1.0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.header.frame_id = frame_id;

  visualization_msgs::Marker marker_pair(marker);
  marker_pair.ns = "intersection_layer_pair";
  marker_pair.color.r = 0.0;
  marker_pair.color.g = 1.0;
  marker_pair.color.b = 1.0;

  for (const auto &block_index : blocks) {
    const auto &block = intersection_layer.getBlockByIndex(block_index);

    for (size_t i = 0; i < block.num_voxels(); ++i) {
      const auto &voxel = block.getVoxelByLinearIndex(i);

      if (voxel.count) {
        auto voxel_index = block.computeVoxelIndexFromLinearIndex(i);
        auto pos = block.origin() + voxel_index.cast<float>() * voxel_size;
        // marker
        geometry_msgs::Point point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();

        ROS_INFO("COUNT %i", voxel.count);

        if (voxel.count % 2) {
          marker_pair.points.push_back(point);
        } else {
          marker.points.push_back(point);
        }
      }
    }
  }

  markers->markers.push_back(marker);
  markers->markers.push_back(marker_pair);
}

void visualizeBIM(const bim::BimMap &map,
                  visualization_msgs::MarkerArray *markers,
                  const std::string &frame_id) {

  for (auto &wall : map.walls) {
    visualization_msgs::Marker wall_marker;
    wall_marker.header.frame_id = frame_id;
    wall_marker.type = visualization_msgs::Marker::CUBE;
    wall_marker.scale.x = (wall.thickness == 0 ? 0.01 : wall.thickness);
    wall_marker.scale.y = wall.length;
    wall_marker.scale.z = wall.height;
    wall_marker.color.a = 1.0;
    wall_marker.color.r = 0.5;
    wall_marker.color.g = 0.5;
    wall_marker.color.b = 0.5;
    wall_marker.ns = "wall " + wall.tag;
    wall_marker.pose.position.x = wall.min.x(); //- wall_marker.scale.x / 2.0;
    wall_marker.pose.position.y = wall.min.y(); //- wall_marker.scale.y / 2.0;
    wall_marker.pose.position.z = wall.min.z(); //- wall_marker.scale.z / 2.0;

    auto theta = std::atan2(wall.nor.y(), wall.nor.x());

    auto aa = Eigen::AngleAxisf(-theta, Eigen::Vector3f::UnitZ());
    auto quat = Eigen::Quaternionf(aa);

    auto R = Eigen::Affine3f(quat);
    auto tr = Eigen::Affine3f(Eigen::Translation3f(
        Eigen::Vector3f{-wall_marker.scale.x / 2.0, wall_marker.scale.y / 2.0,
                        wall_marker.scale.z / 2.0}));

    auto o = (R * tr).translation();

    wall_marker.pose.position.x -= o.x();
    wall_marker.pose.position.y -= o.y();
    wall_marker.pose.position.z += o.z();

    // wall_marker.pose.position.x += wall.nor.x() * wall.thickness / 2;
    // wall_marker.pose.position.y += wall.nor.y() * wall.length / 2;

    wall_marker.pose.orientation.w = quat.w();
    wall_marker.pose.orientation.x = quat.x();
    wall_marker.pose.orientation.y = quat.y();
    wall_marker.pose.orientation.z = quat.z();

    markers->markers.push_back(wall_marker);
  }
}