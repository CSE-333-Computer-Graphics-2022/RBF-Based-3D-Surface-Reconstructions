

#pragma once


#include <vector>
#include <opencv2/core.hpp>

#include <nv/mat.h>
#include <nv/camera.h>
#include <nv/rgbd/pyramid.h>
#include <nv/sparse_voxel_grid.h>


/**
 * @brief   Recolorizing Signed Distance fields from input RGB-D frames
 * @author  Robert Maier <robert.maier@tum.de>
 */

namespace nv
{

    /**
     * @brief   Struct for storing observations of a 3D point in an RGB-D
     *          input frame (color, weight and frame id)
     * @author  Robert Maier <robert.maier@tum.de>
     */
    struct VertexObservation
    {
        VertexObservation() :
            color(Vec3b::Zero()),
            weight(0.0f),
            frame(-1)
        {
        }

        VertexObservation(const VertexObservation &obs) :
            color(obs.color),
            weight(obs.weight),
            frame(obs.frame)
        {
        }

        Vec3b color;
        float weight;
        int frame;

        bool operator<(const VertexObservation &obs) const;
    };



    /**
     * @brief   Class for re-colorizing a Signed Distance Field
     *          from RGB-D input data
     * @author  Robert Maier <robert.maier@tum.de>
     */
    class SDFColorization
	{
	public:

        struct Config
        {
            int discont_distance = 0;
            Vec3b color_unobserved = Vec3b::Zero();
            float color_range = 20.0f;
            float max_occlusion_distance = 0.05f;
            size_t max_num_observations = 5;
        };


        SDFColorization(const Camera &cam,
                        SparseVoxelGrid<VoxelSBR>* grid = nullptr);
        ~SDFColorization();

        bool reset(SparseVoxelGrid<VoxelSBR>* grid);
        bool reset(SparseVoxelGrid<VoxelSBR>* grid, const Vec4 &intrinsics, const Vec5 &dist_coeffs,
                   int w, int h);
        bool reset(SparseVoxelGrid<VoxelSBR>* grid, const Camera &cam);

        void setConfig(const Config& cfg);
        const Config& config() const;

        bool add(int id, const cv::Mat &depth, const cv::Mat &color, const Mat4f &pose_world_to_cam);
        bool compute();

        void collectObservations(const std::vector<Vec6> &poses,
                                 std::vector<Pyramid> &frames_pyr,
                                 const Vec3i &v_pos, const Vec3f &n, int pyr_lvl,
                                 std::vector<VertexObservation> &voxel_observations) const;

        VertexObservation computeObservation(const Vec3i &v_pos, const Vec3f &normal,
                                            const Mat4f &pose_world_to_cam, const cv::Mat &color,
                                            const cv::Mat &depth) const;

        static void filter(std::vector<VertexObservation> &observations, size_t n);

	private:
        bool isVoxelVisible(const Vec3f &pt, const cv::Mat& depth, int x, int y) const;
        float computeWeight(const cv::Mat &depth, const Vec3f &n, const int x, const int y, const Vec3f &v) const;
        Vec3f computeColor(const std::vector<VertexObservation> &verts_obs) const;

        Config cfg_;
        Camera cam_;
        SparseVoxelGrid<VoxelSBR>* grid_;
        std::vector< std::vector<VertexObservation> > voxel_observations_;
	};

} 
