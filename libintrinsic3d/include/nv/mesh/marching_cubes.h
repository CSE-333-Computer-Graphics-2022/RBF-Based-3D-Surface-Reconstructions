#pragma once

#include <nv/mat.h>
#include <nv/mesh.h>
#include <nv/sparse_voxel_grid.h>

#include "omp.h"


namespace nv
{
	template <class T>
	class MarchingCubes
	{
		public:
			static Mesh* extractSurface(const SparseVoxelGrid<T>& grid);

		private:
			MarchingCubes();
			MarchingCubes(const MarchingCubes&);
			MarchingCubes& operator=(const MarchingCubes&);
			~MarchingCubes();

			struct Vertex
			{
				Vec3f p;
				Vec3f c;
			};

			struct Triangle
			{
				Vertex v0;
				Vertex v1;
				Vertex v2;
			};

			Mesh* extractMesh(const SparseVoxelGrid<T>& grid);
			Mesh* merge(const std::vector< std::vector<Triangle> > &results) const;

			void extractSurfaceAt(const Vec3i& voxel, const SparseVoxelGrid<T>& grid, std::vector<Triangle>& result);
			int computeLutIndex(const SparseVoxelGrid<T>& grid, int i, int j, int k, float iso_value);
			Vec3f interpolate(float tsdf0, float tsdf1, const Vec3f &val0, const Vec3f &val1, float iso_value);
			typename MarchingCubes::Vertex getVertex(const SparseVoxelGrid<T>& grid, int i1, int j1, int k1, int i2, int j2, int k2, float iso_value);
			
			const static int edge_table_[256];
			const static int triangle_table_[256][16];
	};
} 