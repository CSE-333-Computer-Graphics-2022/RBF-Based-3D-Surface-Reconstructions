

#pragma once


#include <iostream>
#include <unordered_map>
#include <memory>

#include <nv/mat.h>
#include <nv/camera.h>

#include <opencv2/core.hpp>


/**
 * @brief   Sparse voxel-hashed volumetric Signed Signed Distance Field
 * @author  Robert Maier <robert.maier@tum.de>
 */
namespace nv
{

    /**
     * @brief   Basic voxel struct
     * @author  Robert Maier <robert.maier@tum.de>
     */
	struct Voxel
	{
        Voxel(){}
        float sdf = 0.0f;
        float weight = 0.0f;
        Vec3b color = Vec3b::Zero();
    };


    /**
     * @brief   Voxel struct for SDF refinement
     * @author  Robert Maier <robert.maier@tum.de>
     */
	struct VoxelSBR
	{
        VoxelSBR(){}
        double sdf = 0.0;
        float weight = 0.0f;
        Vec3b color = Vec3b::Zero();
        double albedo = 0.6;
        double sdf_refined = 0.0;
    };


    /**
     * @brief   Class for sparse voxel-hashed SDF
     * @author  Robert Maier <robert.maier@tum.de>
     */
	template <class T>
	class SparseVoxelGrid
	{
	public:
        static SparseVoxelGrid* create(float voxel_size, float depth_min = 0.1f, float depth_max = 10.0f);
        static SparseVoxelGrid* create(const std::string &filename, float depth_min = 0.1f, float depth_max = 10.0f);

		~SparseVoxelGrid();

		typedef typename std::unordered_map<Vec3i, T, std::hash<Vec3i>>::iterator iterator;
		typedef typename std::unordered_map<Vec3i, T, std::hash<Vec3i>>::const_iterator const_iterator;
		iterator begin();
		iterator end();
		const_iterator begin() const;
		const_iterator end() const;

        void setIntegrationWeightSample(float weight);
        void setClipBounds(const Vec6f &clip_bounds);

		float voxelSize() const;
        float truncation() const;

		float depthMin() const;
		float depthMax() const;

        bool integrate(const Camera &depth_cam, Camera &color_cam,
                       const cv::Mat &depth, const cv::Mat &color,
                       const cv::Mat &normals, const Mat4f &pose_camera_to_world);

        T& voxel(const Vec3i &voxel_pos);
        T& voxel(const Vec3f &world_pos);
		T& voxel(int x, int y, int z);
        const T& voxel(const Vec3i &voxel_pos) const;
        const T& voxel(const Vec3f &world_pos) const;
		const T& voxel(int x, int y, int z) const;

        Vec3i worldToVoxel(const Vec3f& p) const;
        Vec3f worldToVoxelFloat(const Vec3f& p) const;
        Vec3f voxelToWorld(const Vec3i& v) const;

        bool exists(int x, int y, int z) const;
        bool exists(const Vec3i &voxel_pos) const;

        bool valid(int x, int y, int z) const;
        bool valid(const Vec3i &voxel_pos) const;

		size_t numVoxels() const;
        void setVoxel(const Vec3i &voxel_pos, const T &voxel);

		bool empty() const;
		void clear();
        bool remove(const Vec3i &voxel_pos);

        void printInfo() const;

		bool save(const std::string& filename) const;

        SparseVoxelGrid<T>* clone() const;

	private:
        SparseVoxelGrid(float voxel_size, float depth_min, float depth_max);
		SparseVoxelGrid(const SparseVoxelGrid&);
		SparseVoxelGrid& operator=(const SparseVoxelGrid&);

        void alloc(const Camera &cam, const cv::Mat &depth, const Mat4f &pose_camera_to_world, const Vec6i &voxel_bounds);

        Vec6i computeFrustumBounds(const Camera& cam, const Mat4f& pose) const;

		bool load(const std::string& filename);

		std::unordered_map<Vec3i, T, std::hash<Vec3i> > data_;
        float voxel_size_;
        float depth_min_;
        float depth_max_;
        float truncation_;
        float integration_weight_sample_;
        Vec6f clip_bounds_;
	};

} 
