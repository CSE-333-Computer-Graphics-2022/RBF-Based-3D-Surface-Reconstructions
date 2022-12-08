

#pragma once


#include <nv/mat.h>
#include <opencv2/core.hpp>


/**
 * @brief   Color space conversion and utility functions
 * @author  Robert Maier <robert.maier@tum.de>
 */
namespace nv
{

	float intensity(unsigned char r, unsigned char g, unsigned char b);
	float intensity(const Vec3b &color);
	float intensity(const Vec3f &color);

	Vec3f chromacity(const Vec3b &color);

    template <typename T>
    Vec3b scalarToColor(const T val, const T scale = 255.0);

    Vec3f checkRange(const Vec3f &vec, const float min_val = 0.0f, const float max_val = 255.0f);

	Vec3b randomColor();

} 
