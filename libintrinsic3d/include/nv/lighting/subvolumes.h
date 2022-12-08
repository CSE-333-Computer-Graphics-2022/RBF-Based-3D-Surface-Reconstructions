#pragma once

#include <nv/mat.h>
#include <string>
#include <unordered_map>
#include <nv/sparse_voxel_grid.h>


namespace nv
{
	class Subvolumes
	{
		public:
			Subvolumes(float size);
			~Subvolumes();

			void clear();
			bool compute(const SparseVoxelGrid<VoxelSBR>* grid);

			float subvolumeSize() const;

			size_t count() const;

			Vec3i index(int subvol) const;
			Vec6i bounds(int subvol) const;
			Vec3b color(int subvol) const;

			bool exists(int subvol) const;
			bool exists(const Vec3i &idx) const;

			Vec3f pointToIndexCoord(const Vec3f &pt) const;
			int pointToSubvolume(const Vec3f &p) const;
			int indexToSubvolume(const Vec3i &idx) const;

			template <class T>
			T interpolate(const std::vector<T> &values, const Vec3f &pt, bool linear = true) const;

		private:
			void generate(const SparseVoxelGrid<VoxelSBR>* grid);

			int indexToVoxel(int idx) const;
			Vec6i indexToBounds(const Vec3i& idx) const;

			float pointToIndexFloat(const float pt) const;
			Vec3f pointToIndexFloat(const Vec3f &pt) const;
			int pointToIndex(const float pt) const;
			Vec3i pointToIndex(const Vec3f &pt) const;

			float size_;
			float voxel_size_;
			std::unordered_map<Vec3i, int, std::hash<Vec3i> > subvolumes_;
			std::vector<Vec3i> indices_;
			std::vector<Vec6i> bounds_;
			std::vector<Vec3b> colors_;
	};

} 