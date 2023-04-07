#pragma once

#include "voxblox_skeleton/bim/map_builder.h"

#include <visualization_msgs/MarkerArray.h>

void visualizeIntersectionLayer(
    const voxblox::Layer<map_builder::IntersectionVoxel> &intersection_layer,
    visualization_msgs::MarkerArray *markers, const std::string &frame_id,
    float alpha = 1.0f);

void visualizeBIM(const bim::BimMap &map,
                  visualization_msgs::MarkerArray *markers,
                  const std::string &frame_id);