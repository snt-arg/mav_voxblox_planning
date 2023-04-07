#include "voxblox_skeleton/bim/bim.h"

#include <Eigen/Geometry>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <vector>

#include <voxblox/utils/neighbor_tools.h>

namespace bim {

geom::Cube Wall::asCube() const {
  // construct a cube from our bim information
  geom::Cube cube;
  Eigen::Vector3f nor3d = {nor.x(), nor.y(), 0};
  Eigen::Vector3f nor_orth_3d = {-nor.y(), nor.x(), 0};

  cube.a = min;
  cube.b = cube.a - nor3d * thickness;
  cube.d = cube.a - nor_orth_3d * length;
  cube.c = cube.d - nor3d * thickness;

  if (thickness < 0) {
    std::swap(cube.a, cube.b);
    std::swap(cube.c, cube.d);
  }

  cube.e = cube.a + geom::Point{0, 0, height};
  cube.f = cube.b + geom::Point{0, 0, height};
  cube.g = cube.c + geom::Point{0, 0, height};
  cube.h = cube.d + geom::Point{0, 0, height};

  cube.aabb.addPoint(cube.a);
  cube.aabb.addPoint(cube.b);
  cube.aabb.addPoint(cube.c);
  cube.aabb.addPoint(cube.d);
  cube.aabb.addPoint(cube.e);
  cube.aabb.addPoint(cube.f);
  cube.aabb.addPoint(cube.g);
  cube.aabb.addPoint(cube.h);

  cube.is_plane = thickness == 0;

  return cube;
}

bool Wall::isPointInside(geom::Point point) const {
  return asCube().isPointInside(point);
}

Json::Value parse_json(const std::string &filename) {
  std::ifstream f(filename);
  Json::Value entities;
  f >> entities;

  return entities;
}

BimMap parse_bim(const std::string &filename, float voxel_size) {
  const auto root = parse_json(filename);

  // create a list of walls
  std::vector<Door> doors;
  std::vector<Wall> walls;
  for (Json::Value::ArrayIndex i = 0; i < root.size(); ++i) {
    const Json::Value entity = root[i];

    // parse entities
    if (entity["type"] == "wall") {
      //  walls

      float thickness = 0;
      if (entity["thickness"].isDouble()) {
        thickness = entity["thickness"].asFloat();

        // thicken?
        if (std::abs(voxel_size) > 0) {
          thickness = std::copysign(std::max(std::abs(thickness), voxel_size),
                                    thickness);
        }
      }

      walls.emplace_back(Wall{
          .min =
              geom::Point{entity["X-min"].asFloat(), entity["Y-min"].asFloat(),
                          entity["Z-min"].asFloat()},
          .nor = Eigen::Vector2f{entity["X-nor"].asFloat(),
                                 entity["Y-nor"].asFloat()}
                     .normalized(),
          .length = entity["Length"].asFloat(),
          .thickness = thickness,
          .height = 2.5f, // TODO: currently not exported
          .tag = entity["TAG"].asString(),
      });
    } else if (entity["type"] == "door") {
      //  doors
      doors.emplace_back(Door{
          .min =
              geom::Point{entity["X-min"].asFloat(), entity["Y-min"].asFloat(),
                          entity["Z-min"].asFloat()},
          .nor = Eigen::Vector2f{entity["X-nor"].asFloat(),
                                 entity["Y-nor"].asFloat()}
                     .normalized(),
          .length = entity["Length"].asFloat(),
          .thickness = entity["thickness"].asFloat(),
          .height = 1.5f, // TODO: currently not exported
          .tag = entity["TAG"].asString(),
      });
    }
  }

  // calculate some basic map information
  // TODO
  BimMap map(walls, doors);

  LOG(INFO) << "Loaded BIM map with '" << map.walls().size() << "' walls, '"
            << map.doors().size() << "' doors";

  return map;
}

BimMap::BimMap(std::vector<Wall> walls, std::vector<Door> doors)
    : walls_(walls), doors_(doors) {
  // cache triangles
  for (const auto &wall : walls_) {
    for (const auto triangle : wall.asCube().triangles()) {
      triangles_.push_back(triangle);
    }
  }

  // cache aabb
  geom::AABB aabb;
  for (const auto &tri : triangles_) {
    aabb.addPoint(tri.a);
    aabb.addPoint(tri.b);
    aabb.addPoint(tri.c);
  }
  aabb_ = aabb;
}

bool BimMap::empty() const { return walls_.empty(); }

const geom::AABB &BimMap::aabb() const { return aabb_; }

const std::vector<geom::Triangle> &BimMap::triangles() const {
  return triangles_;
}

const std::vector<Wall> &BimMap::walls() const { return walls_; }

const std::vector<Door> &BimMap::doors() const { return doors_; }

} // namespace bim