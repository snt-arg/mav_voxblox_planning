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

  cube.is_plane = thickness == 0;

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

bool BimMap::empty() const { return walls.empty(); }

BimMap parse_bim(const std::string &filename) {
  const auto root = parse_json(filename);

  // create a list of walls
  std::vector<Wall> walls;
  for (Json::Value::ArrayIndex i = 0; i < root.size(); ++i) {
    const Json::Value entity = root[i];

    if (!entity["thickness"].isDouble()) {
      continue;
    }

    walls.emplace_back(Wall{

        .min = geom::Point{entity["X-min"].asFloat(), entity["Y-min"].asFloat(),
                           entity["Z-min"].asFloat()},
        .nor = Eigen::Vector2f{entity["X-nor"].asFloat(),
                               entity["Y-nor"].asFloat()},
        .length = entity["Length"].asFloat(),
        .thickness = entity["thickness"].isDouble()
                         ? entity["thickness"].asFloat()
                         : 0.0f, // a plane
        .height = 2.5f,          // TODO: currently not exported
        .tag = entity["TAG"].asString(),

    });
  }

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

std::shared_ptr<voxblox::Layer<IntersectionVoxel>>
generateTsdfLayer(const BimMap &bim_map,
                  voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer) {
  using namespace voxblox;

  std::cout << "Generate tsdf" << std::endl;

  // walls to cubes
  std::vector<geom::Cube> cubes;
  for (const auto &wall : bim_map.walls) {
    cubes.emplace_back(wall.asCube());
  }

  std::cout << "Transformed to cubes" << std::endl;

  // Process
  // 1. Integrate all triangles
  // 2. (optionally) flood-fill unoccupied space
  // 3. update signs

  const auto voxel_size = tsdf_layer.voxel_size();
  const auto voxel_size_inv = tsdf_layer.voxel_size_inv();
  const auto voxels_per_side_inv = tsdf_layer.voxels_per_side_inv();
  const auto voxels_per_side = tsdf_layer.voxels_per_side();

  // create intersection layer
  auto intersection_layer =
      std::make_shared<Layer<IntersectionVoxel>>(voxel_size, voxels_per_side);

  // populate tsdf & intersection layer
  for (auto &cube : cubes) {
    // if (!cube.is_plane)
    //   continue;

    const auto &aabb = cube.aabb;

    for (auto triangle : cube.triangles()) {
      integrateTriangle(triangle, *intersection_layer, tsdf_layer);
    }
  }

  bool fill_inside = false;

  // floodfillUnoccupied(4 * voxel_size, fill_inside, *intersection_layer,
  //                     tsdf_layer);
  // updateSigns(*intersection_layer, tsdf_layer, fill_inside);

  return intersection_layer;
}

void integrateTriangle(const geom::Triangle &triangle,
                       voxblox::Layer<IntersectionVoxel> &intersection_layer,
                       voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer) {
  using namespace voxblox;

  const auto voxel_size = tsdf_layer.voxel_size();
  const auto voxel_size_inv = tsdf_layer.voxel_size_inv();
  const auto voxels_per_side_inv = tsdf_layer.voxels_per_side_inv();
  const auto voxels_per_side = tsdf_layer.voxels_per_side();

  int aabb_padding = 1;

  // Instantiate the triangle geometry tool
  const geom::TriangleGeometer triangle_geometer(triangle);

  // Get the triangle's Axis Aligned Bounding Box
  const auto aabb_tight = triangle_geometer.getAABB();

  // Express the AABB corners in voxel index units
  GlobalIndex aabb_min_index = (aabb_tight.min.array() * voxel_size_inv)
                                   .floor()
                                   .cast<LongIndexElement>();
  GlobalIndex aabb_max_index =
      (aabb_tight.max.array() * voxel_size_inv).ceil().cast<LongIndexElement>();

  // Add padding to the AABB indices
  const GlobalIndex voxel_index_min = aabb_min_index.array() - aabb_padding;
  const GlobalIndex voxel_index_max = aabb_max_index.array() + aabb_padding;

  // Iterate over all voxels within the triangle's padded AABB
  LongIndexElement x, y, z;
  Point voxel_origin;
  for (z = voxel_index_min[2]; z < voxel_index_max[2]; z++) {
    for (y = voxel_index_min[1]; y < voxel_index_max[1]; y++) {
      for (x = voxel_index_min[0]; x < voxel_index_max[0]; x++) {
        GlobalIndex voxel_index(x, y, z);

        // Get the indices of the current voxel and its containing block
        BlockIndex block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(
            voxel_index, voxels_per_side_inv);
        VoxelIndex local_voxel_index =
            voxblox::getLocalFromGlobalVoxelIndex(voxel_index, voxels_per_side);

        // Allocate the block and get the voxel
        voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_layer.allocateBlockPtrByIndex(block_index);
        voxblox::TsdfVoxel &voxel =
            block_ptr->getVoxelByVoxelIndex(local_voxel_index);

        // Compute distance to triangle
        voxel_origin =
            voxblox::getOriginPointFromGridIndex(voxel_index, voxel_size);
        float distance = triangle_geometer.getDistanceToPoint(voxel_origin);

        // Update voxel if new distance is lower or if it is new
        // TODO(victorr): Take the absolute distance, to account for signs that
        //                might already have been computed
        if (std::abs(distance) < std::abs(voxel.distance) ||
            voxel.weight == 0.0f) {
          voxel.distance = distance;
          voxel.weight += 1;
        }
      }

      // Mark intersections
      // NOTE: We check if and where the ray that lies parallel to the x axis
      //       and goes through voxel origin collides with the triangle.
      //       If it does, we increase the intersection counter for the voxel
      //       containing the intersection.
      const geom::Point2D ray(voxel_origin.y(), voxel_origin.z());
      Point barycentric_coordinates;
      bool ray_intersects_triangle =
          triangle_geometer.getRayIntersection(ray, &barycentric_coordinates);
      if (ray_intersects_triangle) {
        // Get the voxel x index at the intersection
        float intersection_x_coordinate =
            barycentric_coordinates[0] * triangle.a[0] +
            barycentric_coordinates[1] * triangle.b[0] +
            barycentric_coordinates[2] * triangle.c[0];
        auto intersection_x_index = static_cast<IndexElement>(
            std::ceil(intersection_x_coordinate * voxel_size_inv));

        // Get the indices of the corresponding voxel and its containing block
        BlockIndex block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(
            GlobalIndex(intersection_x_index, y, z), voxels_per_side_inv);
        VoxelIndex local_voxel_index = voxblox::getLocalFromGlobalVoxelIndex(
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

void floodfillUnoccupied(float distance_value, bool fill_inside,
                         voxblox::Layer<IntersectionVoxel> &intersection_layer,
                         voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer) {
  using namespace voxblox;

  const auto voxel_size = tsdf_layer.voxel_size();
  const auto voxel_size_inv = tsdf_layer.voxel_size_inv();
  const auto voxels_per_side_inv = tsdf_layer.voxels_per_side_inv();
  const auto voxels_per_side = tsdf_layer.voxels_per_side();

  // Get the TSDF AABB, expressed in voxel index units
  auto [global_voxel_index_min, global_voxel_index_max] =
      getAABBIndices(intersection_layer);

  // We're then going to iterate over all of the voxels within the AABB.
  // For any voxel which is unobserved and has free neighbors, mark it as free.
  // This assumes a watertight mesh (which is what's required for this algorithm
  // anyway).
  LongIndexElement x, y, z;
  // Iterate over x, y, z to minimize cache misses.
  for (x = global_voxel_index_min.x(); x < global_voxel_index_max.x(); x++) {
    for (y = global_voxel_index_min.y(); y < global_voxel_index_max.y(); y++) {
      for (z = global_voxel_index_min.z(); z < global_voxel_index_max.z();
           z++) {

        GlobalIndex global_voxel_index(x, y, z);
        voxblox::TsdfVoxel *tsdf_voxel =
            tsdf_layer.getVoxelPtrByGlobalIndex(global_voxel_index);
        // If the block doesn't exist, make it.
        if (tsdf_voxel == nullptr) {
          BlockIndex block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(
              global_voxel_index, voxels_per_side_inv);
          tsdf_layer.allocateBlockPtrByIndex(block_index);
          tsdf_voxel = tsdf_layer.getVoxelPtrByGlobalIndex(global_voxel_index);
        }

        // If this is unobserved, then we need to check its neighbors.
        if (tsdf_voxel != nullptr && tsdf_voxel->weight <= 0.0f) {
          voxblox::GlobalIndexVector neighbors;
          voxblox::Neighborhood<>::IndexMatrix neighbor_indices;
          voxblox::Neighborhood<>::getFromGlobalIndex(global_voxel_index,
                                                      &neighbor_indices);
          for (unsigned int idx = 0u; idx < neighbor_indices.cols(); ++idx) {
            const GlobalIndex &neighbor_index = neighbor_indices.col(idx);
            voxblox::TsdfVoxel *neighbor_voxel =
                tsdf_layer.getVoxelPtrByGlobalIndex(neighbor_index);
            // One free neighbor is enough to mark this voxel as free.
            if (neighbor_voxel != nullptr && neighbor_voxel->weight > 0.0f &&
                neighbor_voxel->distance > 0.0f) {
              tsdf_voxel->distance = distance_value;
              tsdf_voxel->weight = 1.0f;
              break;
            }
          }

          // If this voxel is in the AABB but didn't get updated from a
          // neighbor, assume it's outside the mesh and set the corresponding
          // sign of distance value.
          if (tsdf_voxel->weight <= 0.0f) {
            tsdf_voxel->weight = 1.0f;
            tsdf_voxel->distance =
                fill_inside ? distance_value : -distance_value;
          }
        }
        // Otherwise just move on, we're not modifying anything that's already
        // been updated.
      }
    }
  }
}

void updateSigns(voxblox::Layer<IntersectionVoxel> &intersection_layer,
                 voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer,
                 bool fill_inside) {
  using namespace voxblox;

  LOG(INFO) << "Computing the signs...";
  // Get the TSDF AABB, expressed in voxel index units
  auto [global_voxel_index_min, global_voxel_index_max] =
      getAABBIndices(intersection_layer);

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

        if (intersection_count % 2 == fill_inside) {
          // We're inside the surface
          TsdfVoxel *tsdf_voxel =
              tsdf_layer.getVoxelPtrByGlobalIndex(global_voxel_index);
          if (tsdf_voxel) {
            tsdf_voxel->distance = -std::abs(tsdf_voxel->distance);
          }
        }
      }
    }
  }

  // Indicate that the signs are now up to date
  LOG(INFO) << "Computing signs completed.";
}

} // namespace bim