

#pragma once

#include <nv/mat.h>
#include <vector>

#include <ceres/ceres.h>

#include <nv/refinement/cost.h>
#include <nv/sparse_voxel_grid.h>


namespace nv
{

    /**
     * @brief   Class to generate Intrinsic3D surface stabilization
     *          regularizer residuals
     * @author  Robert Maier <robert.maier@tum.de>
     */
    class SurfaceStabRegularizer
	{
	public:
        SurfaceStabRegularizer(double sdf);
        ~SurfaceStabRegularizer();

        static VoxelResidual create(SparseVoxelGrid<VoxelSBR>* grid, const Vec3i& v_pos);

		template <typename T>
        bool operator()(const T* const sdf_refined, T* residual) const
		{
            residual[0] = sdf_refined[0] - T(sdf_);
			if (residual[0] == NV_INVALID_RESIDUAL)
				residual[0] = T(0.0000001);
			return true;
		}
	private:
		double sdf_;
	};

} 
