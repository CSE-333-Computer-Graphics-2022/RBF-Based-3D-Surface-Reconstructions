

#include <nv/refinement/albedo_regularizer.h>

#include <iostream>


namespace nv
{

    AlbedoRegularizer::AlbedoRegularizer()
	{
	}


    AlbedoRegularizer::~AlbedoRegularizer()
	{
	}


    VoxelResidual AlbedoRegularizer::create(SparseVoxelGrid<VoxelSBR>* grid, const Vec3i& v_pos, const Vec3i& v_pos_nb)
	{
		VoxelResidual r;
        if (!grid->valid(v_pos) || !grid->valid(v_pos_nb))
			return r;

		// retrieve voxels
        VoxelSBR& v = grid->voxel(v_pos);
        VoxelSBR& v_nb = grid->voxel(v_pos_nb);

		// compute chromacity difference
		Vec3f c = v.color.cast<float>() * (1.0f / 255.0f);
        Vec3f c_nb = v_nb.color.cast<float>() * (1.0f / 255.0f);
		float lum = intensity(v.color);
        float lum_nb = intensity(v_nb.color);
        float chroma_diff = ((c / lum) - (c_nb / lum_nb)).norm();
        float lum_diff = 1.0f;
        //chroma_diff = math::robustKernel(chroma_diff);
        chroma_diff = std::max(1.0f - chroma_diff, 0.01f);
		// compute chromacity weight by applying robust kernel on chromacity difference
        double w = static_cast<double>(chroma_diff) * static_cast<double>(lum_diff);
		if (std::isnan(w) || std::isinf(w))
			return r;

		// albedo regularizer cost Ea
        AlbedoRegularizer* ar_cost = new AlbedoRegularizer();
        ceres::CostFunction* ar_cost_function = new ceres::AutoDiffCostFunction<AlbedoRegularizer, 1, 1, 1>(ar_cost);
        r.cost = ar_cost_function;
		r.weight = w;
		// parameters
		r.params.push_back(&(v.albedo));
        r.params.push_back(&(v_nb.albedo));

		return r;
	}

} 
