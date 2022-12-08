

#pragma once

#include <nv/mat.h>
#include <nv/sparse_voxel_grid.h>


/**
 * @brief   Math helper functions
 * @author  Robert Maier <robert.maier@tum.de>
 */
namespace nv
{
namespace math
{

    float robustKernel(float val, float thres = 2.0f);

    bool withinBounds(const Vec6i& bounds, const Vec3i &v_pos);
    bool withinBounds(const Vec6f& bounds, const Vec3i &v_pos);
    bool withinBounds(const Vec6f& bounds, const Vec3f &p);

    template <typename T>
    T average(const float weights[8], const T values[8]);

    void interpolationWeights(const Vec3f& pos, Vec3i coords[8], float weights[8]);

    void computeFrustumPoints(const Camera& cam,
                              const float depth_min, const float depth_max,
                              std::vector<Vec3f> &corner_points);

    Mat4 poseVecAAToMat(const Vec6 &pose_vec_aa);
    Vec6 poseMatToVecAA(const Mat4 &pose);

} // namespace math
} 
