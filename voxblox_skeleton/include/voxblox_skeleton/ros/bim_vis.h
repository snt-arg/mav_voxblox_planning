#pragma once

#include "voxblox_skeleton/bim/bim.h"

#include <visualization_msgs/MarkerArray.h>

void visualizeIntersectionLayer(
    const voxblox::Layer<bim::IntersectionVoxel> &intersection_layer,
    visualization_msgs::MarkerArray *markers, const std::string &frame_id);

void visualizeBIM(const bim::BimMap &map,
                  visualization_msgs::MarkerArray *markers,
                  const std::string &frame_id);