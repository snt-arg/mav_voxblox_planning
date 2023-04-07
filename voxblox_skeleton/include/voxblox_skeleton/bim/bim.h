#pragma once

#include <Eigen/Core>
#include <voxblox/core/layer.h>

#include "voxblox_skeleton/bim/geom.h"

namespace bim {

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

using Door = Wall;

class BimMap {
public:
  BimMap() {}
  BimMap(std::vector<Wall> walls, std::vector<Door> doors);

  const geom::AABB &aabb() const;
  const std::vector<geom::Triangle> &triangles() const;
  const std::vector<Wall> &walls() const;
  const std::vector<Door> &doors() const;
  bool empty() const;

private:
  std::vector<Wall> walls_;
  std::vector<Door> doors_;
  std::vector<geom::Triangle> triangles_;
  geom::AABB aabb_;
};

BimMap parse_bim(const std::string &filename, float voxel_size = 0);

} // namespace bim