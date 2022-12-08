

#pragma once

#include <nv/mat.h>
#include <nv/sparse_voxel_grid.h>


/**
 * @brief   SDF algorithms
 * @author  Robert Maier <robert.maier@tum.de>
 */
namespace nv
{
namespace SDFAlgorithms
{
    SparseVoxelGrid<VoxelSBR>* convert(SparseVoxelGrid<Voxel>* grid);

    std::vector<Vec3i> collectRingNeighborhood(const Vec3i &v_pos);
    std::vector<Vec3i> collectFullNeighborhood(const Vec3i &v_pos, int size = 1);

    template <class T>
    bool interpolate(const SparseVoxelGrid<T>* grid, const Vec3f& voxel_pos, T* voxel_out);

    template <class T>
    SparseVoxelGrid<T>* upsample(const SparseVoxelGrid<T>* grid);

    bool checkVoxelsValid(SparseVoxelGrid<VoxelSBR>* grid, const std::vector<Vec3i> &v_pos_neighbors);

    bool applyRefinedSdf(SparseVoxelGrid<VoxelSBR>* grid);

    template <class T>
    bool correctSDF(SparseVoxelGrid<T>* grid, unsigned int num_iter = 10);

    template <class T>
    bool clearInvalidVoxels(SparseVoxelGrid<T>* grid);

    void clearVoxelsOutsideThinShell(SparseVoxelGrid<VoxelSBR>* grid, double thres_shell);

} // namespace SDFAlgorithms
} 
