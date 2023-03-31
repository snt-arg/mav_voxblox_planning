#include "voxblox_skeleton/bim/bim.h"

#include <Eigen/Geometry>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <vector>

#include <voxblox/utils/neighbor_tools.h>

namespace bim {

struct IntersectionVoxel {
  unsigned int count = 0;
};

geom::Cube Wall::asCube() const {
  // construct a cube from our bim information
  geom::Cube cube;
  Eigen::Vector3f nor3d = {nor.x(), nor.y(), 0};
  Eigen::Vector3f nor_orth_3d = {nor.y(), nor.x(), 0};
  cube.a = min;
  cube.b = min + nor_orth_3d * length;
  cube.c = cube.b + nor3d * thickness;
  cube.d = min + nor3d * thickness;

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

  std::cout << "AABB min" << cube.aabb.min << "AABB max" << cube.aabb.max
            << std::endl;

  return cube;
}

bool Wall::isPointInside(geom::Point point) const {
  // const auto angle = std::atan2(nor.y(), nor.x());
  // auto rot =
  //     Eigen::Affine3f(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
  // auto trans = Eigen::Affine3f(Eigen::Translation3f(min - point));

  // Point point_local = (rot.inverse() * trans).translation();

  // Point max = min + Point{thickness, length, height};

  // return point_local.x() >= min.x() && point_local.y() >= min.y() &&
  //        point_local.z() >= min.z() && point_local.x() < max.x() &&
  //        point_local.y() < max.y() && point_local.z() < max.z();
  return false;
}

Json::Value parse_json(const std::string &filename) {
  std::ifstream f(filename);
  Json::Value entities;
  f >> entities;

  return entities;
}

BimMap parse_bim(const std::string &filename) {
  const auto root = parse_json(filename);

  // create a list of walls
  std::vector<Wall> walls;
  for (Json::Value::ArrayIndex i = 0; i < root.size(); ++i) {
    const Json::Value entity = root[i];

    walls.emplace_back(Wall{
        .min = geom::Point{entity["X-min"].asFloat(), entity["Y-min"].asFloat(),
                           entity["Z-min"].asFloat()},
        .nor = Eigen::Vector2f{entity["X-nor"].asFloat(),
                               entity["Y-nor"].asFloat()},
        .length = entity["Length"].asFloat(),
        .thickness = entity["thickness"].isDouble()
                         ? entity["thickness"].asFloat()
                         : 0.25f,
        .height = 2.5f, // TODO: currently not exported
        .tag = entity["TAG"].asString(),

    });
  }
  // std::transform(
  //     entities.begin(), entities.end(), walls.begin(), [](const auto &entity)
  //     {
  //       Wall wall;
  //       try {
  //         wall.length = entity["Length"].asFloat();
  //         wall.min = {entity["X-min"].asFloat(), entity["Y-min"].asFloat(),
  //                     entity["Z-min"].asFloat()};
  //         wall.nor = {entity["X-nor"].asFloat(), entity["Y-nor"].asFloat()};
  //         wall.thickness = entity["thickness"].asFloat();
  //         wall.tag = entity["TAG"].asString();
  //       } catch () {
  //         std::cerr << "Could not parse wall\n" << entity << std::endl;
  //       }
  //       return wall;
  //     });

  // calculate some basic map information
  // TODO
  BimMap map;
  map.walls = walls;

  printf("Loaded BIM map with %i walls", map.walls.size());

  return map;
}

void getAABBIndices(voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer,
                    voxblox::GlobalIndex *global_voxel_index_min,
                    voxblox::GlobalIndex *global_voxel_index_max) {
  CHECK_NOTNULL(global_voxel_index_min);
  CHECK_NOTNULL(global_voxel_index_max);

  using namespace voxblox;

  *global_voxel_index_min =
      GlobalIndex::Constant(std::numeric_limits<LongIndexElement>::max());
  *global_voxel_index_max =
      GlobalIndex::Constant(std::numeric_limits<LongIndexElement>::min());

  const auto voxel_size = tsdf_layer.voxel_size();
  const auto voxel_size_inv = tsdf_layer.voxel_size_inv();
  const auto voxels_per_side_inv = tsdf_layer.voxels_per_side_inv();
  const auto voxels_per_side = tsdf_layer.voxels_per_side();

  // Indices of each block's local axis aligned min and max corners
  const VoxelIndex local_voxel_index_min(0, 0, 0);
  const VoxelIndex local_voxel_index_max(voxels_per_side, voxels_per_side,
                                         voxels_per_side);

  // Iterate over all allocated blocks in the map
  voxblox::BlockIndexList tsdf_block_list;
  tsdf_layer.getAllAllocatedBlocks(&tsdf_block_list);
  for (const voxblox::BlockIndex &block_index : tsdf_block_list) {
    const GlobalIndex global_voxel_index_in_block_min =
        voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
            block_index, local_voxel_index_min, voxels_per_side);
    const GlobalIndex global_voxel_index_in_block_max =
        voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
            block_index, local_voxel_index_max, voxels_per_side);

    // *global_voxel_index_min =
    //     global_voxel_index_min->cwiseMin(global_voxel_index_in_block_min);
    // *global_voxel_index_max =
    //     global_voxel_index_max->cwiseMax(global_voxel_index_in_block_max);
  }
}

void generateTsdfLayer(const BimMap &bim_map,
                       voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer) {
  using namespace voxblox;

  std::cout << "Generate tsdf" << std::endl;

  // walls to cubes
  std::vector<geom::Cube> cubes;
  for (const auto &wall : bim_map.walls) {
    cubes.emplace_back(wall.asCube());
  }

  std::cout << "Transformed to cubes" << std::endl;

  // create intersection layer
  const auto voxel_size = tsdf_layer.voxel_size();
  const auto voxel_size_inv = tsdf_layer.voxel_size_inv();
  const auto voxels_per_side_inv = tsdf_layer.voxels_per_side_inv();
  const auto voxels_per_side = tsdf_layer.voxels_per_side();

  Layer<IntersectionVoxel> intersection_layer(voxel_size, voxels_per_side);

  // populate tsdf & intersection layer
  for (auto &cube : cubes) {
    const auto &aabb = cube.aabb;

    GlobalIndex voxel_index_min =
        (aabb.min.array() * voxel_size_inv).floor().cast<LongIndexElement>();
    GlobalIndex voxel_index_max =
        (aabb.min.array() * voxel_size_inv).floor().cast<LongIndexElement>();

    LongIndexElement x, y, z;
    for (z = voxel_index_min[2]; z < voxel_index_max[2]; z++) {
      for (y = voxel_index_min[1]; y < voxel_index_max[1]; y++) {
        for (x = voxel_index_min[0]; x < voxel_index_max[0]; x++) {
          GlobalIndex voxel_index(x, y, z);

          BlockIndex block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(
              voxel_index, voxels_per_side_inv);
          VoxelIndex local_voxel_index = voxblox::getLocalFromGlobalVoxelIndex(
              voxel_index, voxels_per_side);

          // allocate new block
          auto block_ptr = tsdf_layer.allocateBlockPtrByIndex(block_index);
          auto &voxel = block_ptr->getVoxelByVoxelIndex(local_voxel_index);

          Point voxel_origin =
              voxblox::getOriginPointFromGridIndex(voxel_index, voxel_size);

          const auto triangles = cube.triangles();

          for (const auto &tri : triangles) {
            geom::TriangleGeometer tg(tri);
            float d = tg.getDistanceToPoint(voxel_origin);

            if (std::abs(d) < std::abs(voxel.distance)) {
              voxel.distance = d;
              voxel.weight += 1;
            }

            const geom::Point2D ray(voxel_origin.y(), voxel_origin.z());
            Point barycentric_coordinates;
            bool ray_intersects_triangle =
                tg.getRayIntersection(ray, &barycentric_coordinates);
            if (ray_intersects_triangle) {
              // Get the voxel x index at the intersection
              float intersection_x_coordinate =
                  barycentric_coordinates[0] * tri.a[0] +
                  barycentric_coordinates[1] * tri.b[0] +
                  barycentric_coordinates[2] * tri.c[0];
              auto intersection_x_index = static_cast<IndexElement>(
                  std::ceil(intersection_x_coordinate * voxel_size_inv));

              // Get the indices of the corresponding voxel and its containing
              // block
              BlockIndex block_index =
                  voxblox::getBlockIndexFromGlobalVoxelIndex(
                      GlobalIndex(intersection_x_index, y, z),
                      voxels_per_side_inv);
              VoxelIndex local_voxel_index =
                  voxblox::getLocalFromGlobalVoxelIndex(
                      GlobalIndex(intersection_x_index, y, z), voxels_per_side);

              // Allocate the block and get the voxel
              voxblox::Block<IntersectionVoxel>::Ptr block_ptr =
                  intersection_layer.allocateBlockPtrByIndex(block_index);
              IntersectionVoxel &intersection_voxel =
                  block_ptr->getVoxelByVoxelIndex(local_voxel_index);

              // Increase the count of intersections for this grid cell
              intersection_voxel.count++;
            }
          }
        }
      }
    }
  }

  LOG(INFO) << "Computing the signs...";
  // Get the TSDF AABB, expressed in voxel index units
  GlobalIndex global_voxel_index_min, global_voxel_index_max;
  getAABBIndices(tsdf_layer, &global_voxel_index_min, &global_voxel_index_max);

  // Iterate over all voxels within the TSDF's AABB
  LongIndexElement x, y, z;
  for (z = global_voxel_index_min.z(); z < global_voxel_index_max.z(); z++) {
    for (y = global_voxel_index_min.y(); y < global_voxel_index_max.y(); y++) {
      size_t intersection_count = 0;
      for (x = global_voxel_index_min.x(); x < global_voxel_index_max.x();
           x++) {

        GlobalIndex global_voxel_index(x, y, z);

        IntersectionVoxel *intersection_voxel =
            intersection_layer.getVoxelPtrByGlobalIndex(global_voxel_index);

        if (intersection_voxel) {
          intersection_count += intersection_voxel->count;
        }

        if (intersection_count % 2 == true) {
          // We're inside the surface
          voxblox::TsdfVoxel *tsdf_voxel =
              tsdf_layer.getVoxelPtrByGlobalIndex(global_voxel_index);
          if (tsdf_voxel) {
            tsdf_voxel->distance = -std::abs(tsdf_voxel->distance);
          }
        }
      }
    }
  }
}

} // namespace bim