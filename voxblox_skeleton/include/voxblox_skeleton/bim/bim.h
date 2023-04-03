#pragma once

#include <Eigen/Core>
#include <voxblox/core/layer.h>

#include "voxblox_skeleton/bim/geom.h"

namespace bim {

struct IntersectionVoxel {
  unsigned int count = 0;
};

struct Wall {
  bool isPointInside(geom::Point point) const;
  geom::Cube asCube() const;

  // basic wall definition exported by BIM software
  geom::Point min;
  Eigen::Vector2f nor;
  float length;
  float thickness;
  float height = 2.5f; // TODO: currently not exported
  std::string tag;
};

struct BimMap {
  std::vector<Wall> walls;
};

BimMap parse_bim(const std::string &filename);

void generateTsdfLayer(const BimMap &bim_map,
                       voxblox::Layer<voxblox::TsdfVoxel> &layer);

void integrateTriangle(const geom::Triangle &triangle,
                       voxblox::Layer<IntersectionVoxel> &intersection_layer,
                       voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer);

void floodfillUnoccupied(float distance_value, bool fill_inside,
                         voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer);

void updateSigns(voxblox::Layer<IntersectionVoxel> &intersection_layer,
                 voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer,
                 bool fill_inside);

std::tuple<voxblox::GlobalIndex, voxblox::GlobalIndex>
getAABBIndices(voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer);

} // namespace bim