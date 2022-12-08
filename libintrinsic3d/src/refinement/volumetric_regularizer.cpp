

#include <nv/refinement/volumetric_regularizer.h>

#include <iostream>

#include <nv/sdf/algorithms.h>


namespace nv
{

    VolumetricRegularizer::VolumetricRegularizer()
	{
	}


    VolumetricRegularizer::~VolumetricRegularizer()
	{
	}


    VoxelResidual VolumetricRegularizer::create(SparseVoxelGrid<VoxelSBR>* grid, const Vec3i& v_pos)
	{
		VoxelResidual r;

		// collect 1-ring neighborhood voxels
        std::vector<Vec3i> v_pos_neighbors = SDFAlgorithms::collectRingNeighborhood(v_pos);
		// use only voxels with valid 1-ring-neighborhood
        if (!SDFAlgorithms::checkVoxelsValid(grid, v_pos_neighbors))
			return r;

		// reduce weight if sdf differences are too large
        double w = 1.0;

		// volumetric regularizer cost Er
        VolumetricRegularizer* vr_cost = new VolumetricRegularizer();
        r.cost = new ceres::AutoDiffCostFunction<VolumetricRegularizer, 1, 1, 1, 1, 1, 1, 1, 1>(vr_cost);
		r.weight = w;
        r.params.push_back(&(grid->voxel(v_pos).sdf_refined));
        r.params.push_back(&(grid->voxel(v_pos_neighbors[0]).sdf_refined));
        r.params.push_back(&(grid->voxel(v_pos_neighbors[1]).sdf_refined));
        r.params.push_back(&(grid->voxel(v_pos_neighbors[2]).sdf_refined));
        r.params.push_back(&(grid->voxel(v_pos_neighbors[3]).sdf_refined));
        r.params.push_back(&(grid->voxel(v_pos_neighbors[4]).sdf_refined));
        r.params.push_back(&(grid->voxel(v_pos_neighbors[5]).sdf_refined));

		return r;
	}

} 
