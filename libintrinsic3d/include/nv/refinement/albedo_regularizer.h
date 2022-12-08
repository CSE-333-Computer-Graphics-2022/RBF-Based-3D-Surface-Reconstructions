

#pragma once

#include <nv/mat.h>
#include <vector>

#include <ceres/ceres.h>

#include <nv/color_util.h>
#include <nv/refinement/cost.h>
#include <nv/sparse_voxel_grid.h>


namespace nv
{

    /**
     * @brief   Class to generate Intrinsic3D albedo regularizer residuals
     * @author  Robert Maier <robert.maier@tum.de>
     */
    class AlbedoRegularizer
	{
	public:
        AlbedoRegularizer();
        ~AlbedoRegularizer();

        static VoxelResidual create(SparseVoxelGrid<VoxelSBR>* grid, const Vec3i& v_pos, const Vec3i& v_pos_nb);

		template <typename T>
		bool operator()(const T* const albedo,
            const T* const albedo_neighbor,
			T* residual) const
		{
            residual[0] = albedo[0] - albedo_neighbor[0];
			return true;
		}
	private:

	};

} 
