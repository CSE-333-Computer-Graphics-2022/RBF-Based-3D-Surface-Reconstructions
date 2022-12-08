

#include <nv/sdf/operators.h>

#include <iostream>
#include <unordered_set>
#include <nv/color_util.h>


namespace nv
{

namespace SDFOperators
{

    Vec3f voxelCenterToIso(const SparseVoxelGrid<VoxelSBR>* grid, const Vec3i& v_coord, const Vec3f &n)
    {
        Vec3f pt = grid->voxelToWorld(v_coord);
        return voxelCenterToIso(pt, n, static_cast<float>(grid->voxel(v_coord).sdf_refined));
    }


    Vec3f voxelCenterToIso(const Vec3f &pt, const Vec3f &n, const float sdf)
    {
        return pt - n * sdf;
    }


    Vec3f computeSurfaceNormal(const SparseVoxelGrid<VoxelSBR>* grid, const Vec3i& voxel_coord)
    {
        const int x = voxel_coord[0];
        const int y = voxel_coord[1];
        const int z = voxel_coord[2];

        // check voxel neighbors
        if (!grid->valid(x, y, z) || !grid->valid(x + 1, y, z) || !grid->valid(x, y + 1, z) || !grid->valid(x, y, z + 1))
            return Vec3f::Zero();

        // compute gradient/normal using forward differences
        Vec3f n;
        float sdf0 = static_cast<float>(grid->voxel(x, y, z).sdf_refined);
        n[0] = static_cast<float>(grid->voxel(x + 1, y, z).sdf_refined) - sdf0;
        n[1] = static_cast<float>(grid->voxel(x, y + 1, z).sdf_refined) - sdf0;
        n[2] = static_cast<float>(grid->voxel(x, y, z + 1).sdf_refined) - sdf0;
        if (n.norm() != 0.0f)
            n.normalize();
        return n;
    }


    float laplacian(const SparseVoxelGrid<VoxelSBR>* grid, const Vec3i& voxel_coord)
    {
        const int x = voxel_coord[0];
        const int y = voxel_coord[1];
        const int z = voxel_coord[2];

        // compute discrete volumetric Laplace operator
        // practical derivation:
        // - second order derivative dxx in x-direction
        //   (central differences of first order derivatives dx_forward and dx_backward):
        //   dxx = (dx_forward - dx_backward) = (d(x+1) - d(x)) - (d(x) - d(x-1)) =
        //       = d(x+1) + d(x-1) - 2*d(x)
        // - other second order derivatives dyy and dzz similarly
        // - Laplacian: L = dxx + dyy + dzz
        float sdf = static_cast<float>(grid->voxel(x, y, z).sdf_refined);
        float dxx = static_cast<float>(grid->voxel(x + 1, y, z).sdf_refined) +
                static_cast<float>(grid->voxel(x - 1, y, z).sdf_refined) - 2.0f * sdf;
        float dyy = static_cast<float>(grid->voxel(x, y + 1, z).sdf_refined) +
                static_cast<float>(grid->voxel(x, y - 1, z).sdf_refined) - 2.0f * sdf;
        float dzz = static_cast<float>(grid->voxel(x, y, z + 1).sdf_refined) +
                static_cast<float>(grid->voxel(x, y, z - 1).sdf_refined) - 2.0f * sdf;
        // compute Laplacian and normalize between [-1.0,1.0]
        float lap = (dxx + dyy + dzz) / grid->truncation();
        return lap;
    }


    Vec3f intensityGradient(const SparseVoxelGrid<VoxelSBR>* grid, const Vec3i& v_pos)
    {
        const int x = v_pos[0];
        const int y = v_pos[1];
        const int z = v_pos[2];
        const Vec3i coords[6] =
        {
            Vec3i(x + 1, y, z),
            Vec3i(x, y + 1, z),
            Vec3i(x, y, z + 1),
            Vec3i(x - 1, y, z),
            Vec3i(x, y - 1, z),
            Vec3i(x, y, z - 1)
        };

        float lum = intensity(grid->voxel(x, y, z).color);
        float lum_x_plus = intensity(grid->voxel(coords[0]).color);
        float lum_y_plus = intensity(grid->voxel(coords[1]).color);
        float lum_z_plus = intensity(grid->voxel(coords[2]).color);

        // forward differences
        float dx = (lum_x_plus - lum);
        float dy = (lum_y_plus - lum);
        float dz = (lum_z_plus - lum);
        if (!grid->exists(coords[0]))
            dx = 0.0f;
        if (!grid->exists(coords[1]))
            dy = 0.0f;
        if (!grid->exists(coords[2]))
            dz = 0.0f;

        return Vec3f(dx, dy, dz);
    }


    double sdfToWeight(double sdf, double truncation)
    {
        double sdf_abs_normalized = std::min(std::abs(sdf), truncation) / truncation;
        double weight = std::min(std::max(1.0 - sdf_abs_normalized, 0.01), 1.0);
        return weight;
    }


} // namespace SDFOperators
} 
