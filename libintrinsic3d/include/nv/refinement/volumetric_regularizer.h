

#pragma once

#include <nv/mat.h>
#include <vector>

#include <ceres/ceres.h>

#include <nv/refinement/cost.h>
#include <nv/sdf/operators.h>
#include <nv/sparse_voxel_grid.h>


namespace nv
{

    /**
     * @brief   Class to generate Intrinsic3D volumetric regularizer residuals
     * @author  Robert Maier <robert.maier@tum.de>
     */
    class VolumetricRegularizer
	{
	public:
        VolumetricRegularizer();
        ~VolumetricRegularizer();

        static VoxelResidual create(SparseVoxelGrid<VoxelSBR>* grid, const Vec3i& v_pos);

		template <typename T>
		bool operator()(const T* const sdf,
                        const T* const sdf_x_plus,
                        const T* const sdf_x_minus,
                        const T* const sdf_y_plus,
                        const T* const sdf_y_minus,
                        const T* const sdf_z_plus,
                        const T* const sdf_z_minus,
                        T* residual) const
		{
			// compute discrete volumetric Laplace operator
            residual[0] = SDFOperators::computeLaplacian(sdf[0], sdf_x_plus[0], sdf_x_minus[0], sdf_y_plus[0], sdf_y_minus[0], sdf_z_plus[0], sdf_z_minus[0]);
			return true;
		}

	private:

	};

} 
