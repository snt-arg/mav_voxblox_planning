#pragma once

#include "voxblox_skeleton/bim/bim.h"

namespace map_builder {
struct IntersectionVoxel {
  unsigned int count = 0;
};

struct BimLayers {
  std::shared_ptr<voxblox::Layer<IntersectionVoxel>> intersection_layer;
  std::shared_ptr<voxblox::Layer<IntersectionVoxel>> freespace_layer;
};

BimLayers generateTsdfLayer(const bim::BimMap &bim_map,
                            voxblox::Layer<voxblox::TsdfVoxel> &layer);

void integrateTriangle(const geom::Triangle &triangle,
                       voxblox::Layer<IntersectionVoxel> &intersection_layer,
                       voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer);

void floodfillUnoccupied(float distance_value, bool fill_inside,
                         voxblox::Layer<IntersectionVoxel> &intersection_layer,
                         voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer);

void fillUnoccupied(float distance_value, const bim::BimMap &map,
                    voxblox::Layer<IntersectionVoxel> &intersection_layer,
                    voxblox::Layer<IntersectionVoxel> &freespace_layer,
                    voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer);

void closeLayer(const bim::BimMap &map,
                voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer, int z_index);

void updateSigns(voxblox::Layer<IntersectionVoxel> &intersection_layer,
                 voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer,
                 bool fill_inside);

void integrateDoors(float distance_value, const bim::BimMap &map,
                    voxblox::Layer<IntersectionVoxel> &intersection_layer,
                    voxblox::Layer<IntersectionVoxel> &freespace_layer,
                    voxblox::Layer<voxblox::TsdfVoxel> &tsdf_layer);

template <typename T>
std::tuple<voxblox::GlobalIndex, voxblox::GlobalIndex>
getAABBIndices(voxblox::Layer<T> &layer) {
  using namespace voxblox;

  const auto voxel_size = layer.voxel_size();
  const auto voxel_size_inv = layer.voxel_size_inv();
  const auto voxels_per_side_inv = layer.voxels_per_side_inv();
  const auto voxels_per_side = layer.voxels_per_side();

  GlobalIndex global_voxel_index_min =
      GlobalIndex::Constant(std::numeric_limits<LongIndexElement>::max());
  GlobalIndex global_voxel_index_max =
      GlobalIndex::Constant(std::numeric_limits<LongIndexElement>::min());

  // Indices of each block's local axis aligned min and max corners
  const VoxelIndex local_voxel_index_min(0, 0, 0);
  const VoxelIndex local_voxel_index_max(voxels_per_side, voxels_per_side,
                                         voxels_per_side);

  // Iterate over all allocated blocks in the map
  voxblox::BlockIndexList block_list;
  layer.getAllAllocatedBlocks(&block_list);
  for (const voxblox::BlockIndex &block_index : block_list) {
    const GlobalIndex global_voxel_index_in_block_min =
        voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
            block_index, local_voxel_index_min, voxels_per_side);
    const GlobalIndex global_voxel_index_in_block_max =
        voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
            block_index, local_voxel_index_max, voxels_per_side);

    global_voxel_index_min =
        global_voxel_index_min.cwiseMin(global_voxel_index_in_block_min);
    global_voxel_index_max =
        global_voxel_index_max.cwiseMax(global_voxel_index_in_block_max);
  }
  return {global_voxel_index_min, global_voxel_index_max};
}
} // namespace map_builder