#pragma once

#include <vector>
#include <nv/mat.h>
#include <nv/sparse_voxel_grid.h>
#include <nv/lighting/subvolumes.h>


namespace nv
{
    class LightingSVSH
	{
        public:
            LightingSVSH(const SparseVoxelGrid<VoxelSBR>* grid, float subvolume_size, double lambda_reg, double thres_shell = 0.0, bool weighted = false);
            ~LightingSVSH();

            bool estimate();

            const Subvolumes& subvolumes() const;

            std::vector<Eigen::VectorXd> shCoeffs() const;

            bool interpolate(const Vec3i &v_pos, Eigen::VectorXd &sh_coeffs) const;

            bool computeVoxelShCoeffs(std::vector<Eigen::VectorXd> &voxel_coeffs) const;

        protected:

            const SparseVoxelGrid<VoxelSBR>* grid_;
            float subvolume_size_;
            double thres_shell_;
            bool weighted_;
            double lambda_reg_;
            Subvolumes subvolumes_;
            std::vector<Eigen::VectorXd> sh_coeffs_;
	};

}