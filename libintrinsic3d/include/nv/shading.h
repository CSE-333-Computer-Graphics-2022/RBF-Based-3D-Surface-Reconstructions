

#pragma once

#include <vector>
#include <nv/mat.h>
#include <nv/sdf/operators.h>


/**
 * @brief   Functionality for computing shading
 * @author  Robert Maier <robert.maier@tum.de>
 */

namespace nv
{
namespace Shading
{

    static const int NUM_SPHERICAL_HARMONICS = 9;

    Eigen::VectorXf shBasisFunctions(const Vec3f &n);

    template <typename T>
    inline bool shBasisFunctions(const T* const n, T* sh)
    {
        // compute spherical harmonics basis functions
        sh[0] = T(1.0f);
        sh[1] = n[1];
        sh[2] = n[2];
        sh[3] = n[0];
        sh[4] = n[0] * n[1];
        sh[5] = n[1] * n[2];
        sh[6] = (-n[0] * n[0]) - (n[1] * n[1]) + T(2.0) * (n[2] * n[2]);
        sh[7] = n[0] * n[2];
        sh[8] = (n[0] * n[0]) - (n[1] * n[1]);
        return true;
    }


    float computeShading(const Vec3f &normal, const Eigen::VectorXf &sh_coeffs, float albedo);


    template <typename T>
    inline bool computeShading(const Eigen::VectorXd &sh_coeffs, const T normal[3], const T albedo, T* shading)
    {
        // compute spherical harmonics basis functions
        T sh_funcs[9];
        if (!shBasisFunctions(normal, sh_funcs))
        {
            shading[0] = T(0.0);
            return false;
        }

        // compute shading
        T shad = T(0.0);
        for (int i = 0; i < 9; i++)
            shad += T(sh_coeffs[i]) * T(sh_funcs[i]);
        shading[0] = albedo * shad;

        return true;
    }


    template <typename T>
    inline bool computeShading(const T* sh_coeffs, const T normal[3], const T albedo, T* shading)
    {
        // compute spherical harmonics basis functions
        T sh_funcs[9];
        if (!shBasisFunctions(normal, sh_funcs))
        {
            shading[0] = T(0.0);
            return false;
        }

        // compute shading
        T shad = T(0.0);
        for (int i = 0; i < 9; i++)
            shad += sh_coeffs[i] * T(sh_funcs[i]);
        shading[0] = albedo * shad;

        return true;
    }


    template <typename T>
    inline bool computeShading(const Eigen::VectorXd &sh_coeffs, const T sdf,
                               const T sdf_x_plus, const T sdf_y_plus, const T sdf_z_plus,
                               const T albedo, T* shading)
    {
        // compute surface normal
        T normal[3];
        SDFOperators::computeNormal(sdf, sdf_x_plus, sdf_y_plus, sdf_z_plus, normal);

        return computeShading(sh_coeffs, normal, albedo, shading);
    }


    template <typename T>
    inline T computeShadingGradientDifference(const T lum[3], const T shading[3])
    {
        T dx_lum = lum[1] - lum[0];
        T dy_lum = lum[2] - lum[0];
        T dz_lum = lum[3] - lum[0];

        // compute shading gradients
        T dx_shading = shading[1] - shading[0];
        T dy_shading = shading[2] - shading[0];
        T dz_shading = shading[3] - shading[0];

        // compute shading gradient difference
        T diff_x = dx_shading - dx_lum;
        T diff_y = dy_shading - dy_lum;
        T diff_z = dz_shading - dz_lum;

        // compute scalar of difference vector
        T diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
        return diff;
    }

} // namespace Shading
} 
