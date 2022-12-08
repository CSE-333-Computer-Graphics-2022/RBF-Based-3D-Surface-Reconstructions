

#pragma once

#include <nv/mat.h>
#include <string>
#include <unordered_set>
#include <vector>
#include <opencv2/core.hpp>

#include <nv/settings.h>
#include <nv/sparse_voxel_grid.h>
#include <nv/refinement/nls_solver.h>
#include <nv/rgbd/pyramid.h>
#include <nv/refinement/shading_cost.h>
#include <nv/refinement/cost.h>
#include <nv/sdf/colorization.h>



namespace nv
{
    class NLSSolver;


    /**
     * @brief   Class for joint optimization
     * @author  Robert Maier <robert.maier@tum.de>
     */
    class Optimizer
	{
    public:

        /**
         * @brief   Optimizer config struct for optimization and solver parameters
         * @author  Robert Maier <robert.maier@tum.de>
         */
        struct Config
        {
            int iterations = 10;
            int lm_steps = 50;
            double lambda_g = 0.2;
            double lambda_r0 = 20.0;
            double lambda_r1 = 160.0;
            double lambda_s0 = 10.0;
            double lambda_s1 = 120.0;
            double lambda_a = 0.1;
            bool fix_poses = false;
            bool fix_intrinsics = false;
            bool fix_distortion = false;

            void load(const Settings& cfg);

            void print() const;
        };


        /**
         * @brief   Struct for storing optimizer data
         * @author  Robert Maier <robert.maier@tum.de>
         */
        struct Data
        {
            SparseVoxelGrid<VoxelSBR>* grid = nullptr;
            double thres_shell = 0.0;
            int grid_level = 0;
            int rgbd_level = 0;
            std::vector<Eigen::VectorXd> voxel_sh_coeffs;
            std::vector<ShadingCostData> shading_cost_data;
            std::unordered_set<Vec3i, std::hash<Vec3i> > voxels_added;
        };


        /**
         * @brief   Struct for storing image formation model
         * @author  Robert Maier <robert.maier@tum.de>
         */
        struct ImageFormationModel
        {
            Vec4 intrinsics = Vec4::Zero();
            Vec5 distortion_coeffs = Vec5::Zero(5);

            std::vector<int> frame_ids;
            std::vector<Vec6> poses;
            std::vector<Pyramid> rgbd_pyr;
        };


        Optimizer(Config cfg);
        ~Optimizer();

        const Config& config() const;

        bool optimize(SDFColorization &colorization,
                      Data &data,
                      ImageFormationModel &image_formation);

	private:
        bool addVoxelResiduals(NLSSolver &solver,
                               SDFColorization &colorization,
                               Data &data,
                               ImageFormationModel &image_formation,
                               const Vec3i &v_pos, size_t voxel_idx, int pyr_lvl);

        bool buildProblem(NLSSolver &solver,
                          Data &data,
                          ImageFormationModel &image_formation);

        void fixVoxelParams(NLSSolver &solver, Data &data, bool fix_ring_neighborhood = true);

        Config cfg_;
	};

} 
