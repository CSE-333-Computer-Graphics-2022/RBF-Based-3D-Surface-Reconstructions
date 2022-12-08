

#include <nv/shading.h>

#include <iostream>
#include <vector>


namespace nv
{
namespace Shading
{

    Eigen::VectorXf shBasisFunctions(const Vec3f &n)
    {
        // compute spherical harmonics basis functions
        Eigen::VectorXf sh(NUM_SPHERICAL_HARMONICS);
        if (!shBasisFunctions(n.data(), sh.data()))
            sh.setZero();

        bool sh_valid = true;
        for (int i = 0; i < NUM_SPHERICAL_HARMONICS; ++i)
            if (std::isnan(sh[i]) || std::isinf(sh[i]))
                sh_valid = false;
        if (!sh_valid)
            sh.setZero(NUM_SPHERICAL_HARMONICS);

        return sh;
    }


    float computeShading(const Vec3f &normal, const Eigen::VectorXf &sh_coeffs, float albedo)
    {
        float shad = 0.0f;
        if (normal.norm() != 0.0f && !std::isnan(normal.norm()))
        {
            Eigen::VectorXf sh_basis_funcs = Shading::shBasisFunctions(normal);
            if (sh_basis_funcs.norm() != 0.0f && albedo != 0.0f && !std::isnan(albedo))
            {
                shad = albedo * sh_coeffs.dot(sh_basis_funcs);
            }
        }
        return shad;
    }

} // namespace Shading
} 
