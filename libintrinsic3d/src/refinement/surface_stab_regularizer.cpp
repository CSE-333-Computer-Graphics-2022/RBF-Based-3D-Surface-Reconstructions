

#include <nv/refinement/surface_stab_regularizer.h>

#include <iostream>


namespace nv
{

    SurfaceStabRegularizer::SurfaceStabRegularizer(double sdf) :
		sdf_(sdf)
	{
	}


    SurfaceStabRegularizer::~SurfaceStabRegularizer()
	{
	}


    VoxelResidual SurfaceStabRegularizer::create(SparseVoxelGrid<VoxelSBR>* grid, const Vec3i& v_pos)
	{
        SurfaceStabRegularizer* ss_cost = new SurfaceStabRegularizer(grid->voxel(v_pos).sdf);
        ceres::CostFunction* ss_cost_function = new ceres::AutoDiffCostFunction<SurfaceStabRegularizer, 1, 1>(ss_cost);

		VoxelResidual r;
        r.cost = ss_cost_function;
		r.weight = 1.0;
        r.params.push_back(&(grid->voxel(v_pos).sdf_refined));
		return r;
	}

} 
