

#pragma once

#include <vector>
#include <nv/mat.h>
#include <opencv2/core.hpp>

namespace nv
{

    /**
     * @brief   Triangle mesh container
     * @author  Robert Maier <robert.maier@tum.de>
     */
    struct Mesh
	{
        std::vector<Vec3f> vertices;
        std::vector<Vec3f> normals;
        std::vector<Vec3b> colors;
        std::vector<Vec3i> face_vertices;

        /**
         * @brief Save mesh to .ply file
         * @param filename  output filename
         * @return true if mesh could be saved successfully
         */
        bool save(const std::string &filename) const;

	};

} 
