

#include <nv/color_util.h>

#include <iostream>
#include <ctime>


namespace nv
{

	float intensity(unsigned char r, unsigned char g, unsigned char b)
	{
        return 0.299f * static_cast<float>(r) +
                0.587f * static_cast<float>(g) +
                0.114f * static_cast<float>(b);
	}


	float intensity(const Vec3b &color)
	{
		return intensity(color[0], color[1], color[2]);
	}


	float intensity(const Vec3f &color)
	{
		return 0.299f * color[0] + 0.587f * color[1] + 0.114f * color[2];
	}


	Vec3f chromacity(const Vec3b &color)
	{
		Vec3f c = color.cast<float>();
		float lum = intensity(color);
		Vec3f chrom = c * (1.0f / std::max(lum, 0.001f));
		return chrom;
	}


    template <typename T>
    Vec3b scalarToColor(const T val, const T scale)
    {
		const T min_val = static_cast<T>(0.0);
        const T max_val = static_cast<T>(255.0);
        const T color_val = std::min(std::max(val * scale, min_val), max_val);
        unsigned char c = static_cast<unsigned char>(color_val);
        return Vec3b(c, c, c);
    }
    template Vec3b scalarToColor(const float val, const float scale);
    template Vec3b scalarToColor(const double val, const double scale);


    Vec3f checkRange(const Vec3f &vec, const float min_val, const float max_val)
	{
		Vec3f v;
        v[0] = std::min(std::max(vec[0], min_val), max_val);
        v[1] = std::min(std::max(vec[1], min_val), max_val);
        v[2] = std::min(std::max(vec[2], min_val), max_val);
		return v;
	}


	double rand()
	{
		static bool init = false;
		if (!init)
		{
            init = true;
            std::srand(std::clock());
		}

		double r = double(std::rand()) / RAND_MAX;
		return r;
	}


    Vec3b randomColor()
	{
		Vec3f c;
        for (int i = 0; i < 3; ++i)
            c[i] = static_cast<float>(rand());
        c = checkRange(c * 255.0f);
		return c.cast<unsigned char>();
	}

} 
