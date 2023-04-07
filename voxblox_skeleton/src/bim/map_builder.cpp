#include "voxblox_skeleton/bim/map_builder.h"
#include <voxblox/utils/neighbor_tools.h>

namespace map_builder {
BimLayers generateTsdfLayer(const bim::BimMap &bim_map,
                            voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer) {
  // Process
  // 1. Integrate all triangles
  // 2. fill unoccupied space
  // 3. update signs

  using namespace voxblox;

  LOG(INFO) << "Generating voxels from BIM...";

  const auto voxel_size = tsdf_layer.voxel_size();
  const auto voxel_size_inv = tsdf_layer.voxel_size_inv();
  const auto voxels_per_side_inv = tsdf_layer.voxels_per_side_inv();
  const auto voxels_per_side = tsdf_layer.voxels_per_side();

  // create intersection layer
  auto intersection_layer =
      std::make_shared<Layer<IntersectionVoxel>>(voxel_size, voxels_per_side);

  // create free space layer
  auto freespace_layer =
      std::make_shared<Layer<IntersectionVoxel>>(voxel_size, voxels_per_side);

  // populate tsdf & intersection layer

  LOG(INFO) << "Integrating " << bim_map.triangles().size() << " triangles...";
  for (auto triangle : bim_map.triangles()) {
    integrateTriangle(triangle, *intersection_layer, tsdf_layer);
  }

  LOG(INFO) << "Finding freespace...";
  fillUnoccupied(4 * voxel_size, bim_map, *intersection_layer, *freespace_layer,
                 tsdf_layer);

  LOG(INFO) << "Compute signs...";
  updateSigns(*intersection_layer, tsdf_layer, true);

  LOG(INFO) << "Integrating doors...";
  integrateDoors(4 * voxel_size, bim_map, *intersection_layer, *freespace_layer,
                 tsdf_layer);

  // closeLayer(bim_map, tsdf_layer, 2);
  // closeLayer(bim_map, tsdf_layer, 3);
  // closeLayer(bim_map, tsdf_layer, 4);
  // closeLayer(bim_map, tsdf_layer, 16);
  // closeLayer(bim_map, tsdf_layer, 17);
  // closeLayer(bim_map, tsdf_layer, 18);

  return BimLayers{
      .intersection_layer = intersection_layer,
      .freespace_layer = freespace_layer,
  };
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
        if (std::abs(distance) < std::abs(voxel.distance) ||
            voxel.weight == 0.0f) {
          voxel.distance = distance;
          voxel.weight += 1;
        }
        // {
        //   // Allocate the block and get the voxel
        //   auto block_ptr =
        //       intersection_layer.allocateBlockPtrByIndex(block_index);
        //   auto &voxel = block_ptr->getVoxelByVoxelIndex(local_voxel_index);
        //   voxel.count = 1;
        // }
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

void closeLayer(const bim::BimMap &map,
                voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer, int z_index) {
  using namespace voxblox;

  const auto voxel_size = tsdf_layer.voxel_size();
  const auto voxel_size_inv = tsdf_layer.voxel_size_inv();
  const auto voxels_per_side_inv = tsdf_layer.voxels_per_side_inv();
  const auto voxels_per_side = tsdf_layer.voxels_per_side();

  // Get the TSDF AABB, expressed in voxel index units
  auto [global_voxel_index_min, global_voxel_index_max] =
      getAABBIndices(tsdf_layer);

  LongIndexElement x, y;
  // Iterate over x, y, z to minimize cache misses.
  for (x = global_voxel_index_min.x(); x < global_voxel_index_max.x(); x++) {
    for (y = global_voxel_index_min.y(); y < global_voxel_index_max.y(); y++) {
      BlockIndex block_index;
      VoxelIndex voxel_index;
      voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
          {x, y, z_index}, voxels_per_side, &block_index, &voxel_index);

      // update tsdf
      auto block = tsdf_layer.allocateBlockPtrByIndex(block_index);
      auto &voxel = block->getVoxelByVoxelIndex(voxel_index);
      voxel.distance = 0;
      voxel.weight = 0;
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
  // For any voxel which is unobserved and has free neighbors, mark it as
  // free. This assumes a watertight mesh (which is what's required for this
  // algorithm anyway).
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

void fillUnoccupied(float distance_value, const bim::BimMap &map,
                    voxblox::Layer<IntersectionVoxel> &intersection_layer,
                    voxblox::Layer<IntersectionVoxel> &freespace_layer,
                    voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer) {
  using namespace voxblox;

  const auto voxel_size = tsdf_layer.voxel_size();
  const auto voxel_size_inv = tsdf_layer.voxel_size_inv();
  const auto voxels_per_side_inv = tsdf_layer.voxels_per_side_inv();
  const auto voxels_per_side = tsdf_layer.voxels_per_side();

  // collect all triangles
  std::vector<geom::Triangle> triangles = map.triangles();

  // get the AABB
  const auto map_aabb = map.aabb();
  const auto global_voxel_index_min = GlobalIndex{
      (map_aabb.min * voxel_size_inv).array().floor().cast<LongIndexElement>()};
  const auto global_voxel_index_max = GlobalIndex{
      (map_aabb.max * voxel_size_inv).array().ceil().cast<LongIndexElement>()};

  // sweep along x
  LongIndexElement x, y, z;
  for (y = global_voxel_index_min.y(); y < global_voxel_index_max.y(); y++) {
    for (z = global_voxel_index_min.z(); z < global_voxel_index_max.z(); z++) {
      for (x = global_voxel_index_min.x(); x < global_voxel_index_max.x();
           x++) {

        auto inter_voxel =
            intersection_layer.getVoxelPtrByGlobalIndex({x, y, z});

        if (inter_voxel && inter_voxel->count > 0) {
          continue;
        }

        const GlobalIndex global_voxel_index(x, y, z);
        const GlobalIndex global_voxel_origin_index(
            global_voxel_index_min.x() - 10, y, z);

        const auto ray_origin = voxblox::getCenterPointFromGridIndex(
            global_voxel_origin_index, voxel_size);
        const auto ray_target = voxblox::getCenterPointFromGridIndex(
            global_voxel_index, voxel_size);
        const auto ray_normal = (ray_target - ray_origin).normalized();

        const auto intersections =
            geom::getIntersections(triangles, ray_origin, ray_target);

        if (!intersections.empty() &&
            intersections.back().normal.dot(ray_normal) > 0) {
          BlockIndex block_index;
          VoxelIndex voxel_index;
          voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
              {x, y, z}, voxels_per_side, &block_index, &voxel_index);
          {
            // update esdf
            auto block = tsdf_layer.allocateBlockPtrByIndex(block_index);
            auto &voxel = block->getVoxelByVoxelIndex(voxel_index);
            voxel.distance = distance_value;
            voxel.weight = 1.0;
          }
          {
            // free space
            auto block = freespace_layer.allocateBlockPtrByIndex(block_index);
            auto &voxel = block->getVoxelByVoxelIndex(voxel_index);
            voxel.count = 1;
          }
        }
      }
    }
  }
}

void integrateDoors(float distance_value, const bim::BimMap &map,
                    voxblox::Layer<IntersectionVoxel> &intersection_layer,
                    voxblox::Layer<IntersectionVoxel> &freespace_layer,
                    voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer) {

  using namespace voxblox;

  const auto voxel_size = tsdf_layer.voxel_size();
  const auto voxel_size_inv = tsdf_layer.voxel_size_inv();
  const auto voxels_per_side_inv = tsdf_layer.voxels_per_side_inv();
  const auto voxels_per_side = tsdf_layer.voxels_per_side();

  // get the AABB
  const auto map_aabb = map.aabb();
  const auto global_voxel_index_min =
      GlobalIndex{(map_aabb.min * voxel_size_inv).cast<LongIndexElement>()};
  const auto global_voxel_index_max =
      GlobalIndex{(map_aabb.max * voxel_size_inv).cast<LongIndexElement>()};

  for (const auto &door : map.doors()) {
    const auto door_cube = door.asCube();
    const auto &door_aabb = door_cube.aabb;

    GlobalIndex door_min_index =
        voxblox::getGridIndexFromOriginPoint<GlobalIndex>(
            voxblox::Point{door_aabb.min.x(), door_aabb.min.y(),
                           door_aabb.min.z()},
            voxel_size_inv);

    GlobalIndex door_max_index =
        voxblox::getGridIndexFromOriginPoint<GlobalIndex>(
            voxblox::Point{door_aabb.max.x(), door_aabb.max.y(),
                           door_aabb.max.z()},
            voxel_size_inv);

    LongIndexElement x, y, z;
    for (y = door_min_index.y(); y < door_max_index.y(); y++) {
      for (z = door_min_index.z(); z < door_max_index.z(); z++) {
        for (x = door_min_index.x(); x < door_max_index.x(); x++) {
          auto voxel_center = voxblox::getCenterPointFromGridIndex(
              GlobalIndex{x, y, z}, voxel_size);

          if (door.isPointInside(voxel_center)) {
            BlockIndex block_index;
            VoxelIndex voxel_index;
            voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
                {x, y, z}, voxels_per_side, &block_index, &voxel_index);

            // free space
            {
              auto block = freespace_layer.allocateBlockPtrByIndex(block_index);
              auto &voxel = block->getVoxelByVoxelIndex(voxel_index);
              voxel.count = 1;
            }
            // update esdf
            {
              auto block = tsdf_layer.allocateBlockPtrByIndex(block_index);
              auto &voxel = block->getVoxelByVoxelIndex(voxel_index);
              voxel.distance = distance_value;
              voxel.weight = 1.0;
            }
            // update intersection layer
            {
              auto block =
                  intersection_layer.allocateBlockPtrByIndex(block_index);
              auto &voxel = block->getVoxelByVoxelIndex(voxel_index);
              voxel.count = 0;
            }
          }
        }
      }
    }
  }
}

void updateSigns(voxblox::Layer<IntersectionVoxel> &intersection_layer,
                 voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer,
                 bool fill_inside) {
  using namespace voxblox;

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
}
} // namespace map_builder