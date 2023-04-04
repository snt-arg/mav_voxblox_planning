#include "voxblox_skeleton/ros/bim_vis.h"

#include <Eigen/Geometry>
#include <ros/console.h>

void visualizeIntersectionLayer(
    const voxblox::Layer<bim::IntersectionVoxel> &intersection_layer,
    visualization_msgs::MarkerArray *markers, const std::string &frame_id,
    float alpha) {
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
  marker.color.a = alpha;
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

        if (voxel.count % 2) {
          marker_pair.points.push_back(point);
        } else {
          marker.points.push_back(point);
        }
      }
    }
  }

  if (!marker.points.empty())
    markers->markers.push_back(marker);
  if (!marker_pair.points.empty())
    markers->markers.push_back(marker_pair);
}

void visualizeBIM(const bim::BimMap &map,
                  visualization_msgs::MarkerArray *markers,
                  const std::string &frame_id) {

  for (auto &wall : map.walls()) {
    visualization_msgs::Marker wall_marker;
    wall_marker.header.frame_id = frame_id;
    wall_marker.type = visualization_msgs::Marker::CUBE;
    wall_marker.scale.x = (wall.thickness == 0 ? 0.01 : wall.thickness);
    wall_marker.scale.y = wall.length;
    wall_marker.scale.z = wall.height;
    wall_marker.color.a = 1.0f;
    wall_marker.color.r = 0.5f;
    wall_marker.color.g = 0.5f;
    wall_marker.color.b = 0.5f;
    wall_marker.ns = "wall " + wall.tag;
    wall_marker.pose.position.x = wall.min.x();
    wall_marker.pose.position.y = wall.min.y();
    wall_marker.pose.position.z = wall.min.z();

    auto theta = std::atan2(wall.nor.y(), wall.nor.x());

    auto aa = Eigen::AngleAxisf(-theta, Eigen::Vector3f::UnitZ());
    auto quat = Eigen::Quaternionf(aa);

    auto R = Eigen::Affine3f(quat);
    auto tr =
        Eigen::Vector4d{-wall_marker.scale.x / 2.0, wall_marker.scale.y / 2.0,
                        wall_marker.scale.z / 2.0, 0.0}
            .cast<float>();

    auto o = R.matrix() * tr;

    wall_marker.pose.position.x -= o.x();
    wall_marker.pose.position.y -= o.y();
    wall_marker.pose.position.z += o.z();

    // wall_marker.pose.position.x += wall.nor.x() * wall.thickness / 2;
    // wall_marker.pose.position.y += wall.nor.y() * wall.length / 2;

    wall_marker.pose.orientation.w = quat.w();
    wall_marker.pose.orientation.x = quat.x();
    wall_marker.pose.orientation.y = quat.y();
    wall_marker.pose.orientation.z = quat.z();

    visualization_msgs::Marker wall_marker_norm;
    wall_marker_norm.type = visualization_msgs::Marker::ARROW;
    wall_marker_norm.ns = wall.tag + "norm";
    geometry_msgs::Point start;
    start.x = wall.min.x() - o.x();
    start.y = wall.min.y() - o.y();
    start.z = wall.min.z() + o.z();

    geometry_msgs::Point end;
    end.x = wall.min.x() + wall.nor.x() * 1.5f - o.x();
    end.y = wall.min.y() + wall.nor.y() * 1.5f - o.y();
    end.z = wall.min.z() + o.z();

    ROS_INFO("x %f y %f z %f", o.x(), o.y(), o.z());

    wall_marker_norm.points.push_back(start);
    wall_marker_norm.points.push_back(end);
    wall_marker_norm.header.frame_id = frame_id;
    wall_marker_norm.pose.orientation.w = 1.0;
    wall_marker_norm.scale.x = 0.1f;
    wall_marker_norm.scale.y = 0.1f;
    wall_marker_norm.scale.z = 0.1f;
    wall_marker_norm.color.a = 1.0f;
    wall_marker_norm.color.r = 1.0f;

    // markers->markers.push_back(wall_marker);
    // markers->markers.push_back(wall_marker_norm);

    // wireframe walls
    auto cube = wall.asCube();
    visualization_msgs::Marker wall_wireframe;
    wall_wireframe.header.frame_id = frame_id;
    wall_wireframe.type = visualization_msgs::Marker::LINE_LIST;
    wall_wireframe.ns = wall.tag + "wireframe";
    wall_wireframe.pose.orientation.w = 1.0;
    wall_wireframe.scale.x = 0.01f;
    wall_wireframe.scale.y = 0.01f;
    wall_wireframe.scale.z = 0.01f;
    wall_wireframe.color.a = 1.0f;
    wall_wireframe.color.r = 1.0f;

    visualization_msgs::Marker wall_wireframe_normals;
    wall_wireframe_normals.header.frame_id = frame_id;
    wall_wireframe_normals.type = visualization_msgs::Marker::LINE_LIST;
    wall_wireframe_normals.ns = wall.tag + "wireframe-normal";
    wall_wireframe_normals.pose.orientation.w = 1.0;
    wall_wireframe_normals.scale.x = 0.01f;
    wall_wireframe_normals.scale.y = 0.01f;
    wall_wireframe_normals.scale.z = 0.01f;
    wall_wireframe_normals.color.a = 1.0f;
    wall_wireframe_normals.color.g = 1.0f;
    wall_wireframe_normals.color.b = 1.0f;

    for (auto tri : cube.triangles()) {
      geometry_msgs::Point a;
      a.x = tri.a.x();
      a.y = tri.a.y();
      a.z = tri.a.z();

      geometry_msgs::Point b;
      b.x = tri.b.x();
      b.y = tri.b.y();
      b.z = tri.b.z();

      geometry_msgs::Point c;
      c.x = tri.c.x();
      c.y = tri.c.y();
      c.z = tri.c.z();

      wall_wireframe.points.reserve(6);
      wall_wireframe.points.push_back(a);
      wall_wireframe.points.push_back(b);
      wall_wireframe.points.push_back(b);
      wall_wireframe.points.push_back(c);
      wall_wireframe.points.push_back(c);
      wall_wireframe.points.push_back(a);

      // triangle normal
      auto center = tri.center();
      auto normal = tri.normal() * 0.1;

      geometry_msgs::Point na;
      na.x = center.x();
      na.y = center.y();
      na.z = center.z();

      geometry_msgs::Point nb;
      nb.x = (center + normal).x();
      nb.y = (center + normal).y();
      nb.z = (center + normal).z();

      wall_wireframe_normals.points.reserve(2);
      wall_wireframe_normals.points.push_back(na);
      wall_wireframe_normals.points.push_back(nb);
    }

    if (!wall_wireframe.points.empty())
      markers->markers.push_back(wall_wireframe);
    if (!wall_wireframe_normals.points.empty())
      markers->markers.push_back(wall_wireframe_normals);
  }
}