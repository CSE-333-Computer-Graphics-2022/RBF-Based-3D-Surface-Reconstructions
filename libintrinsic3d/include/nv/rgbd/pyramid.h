

#pragma once

#include <nv/mat.h>
#include <vector>

#include <opencv2/core.hpp>


namespace nv
{

    /**
     * @brief   RGB-D frame pyramid container
     * @author  Robert Maier <robert.maier@tum.de>
     */
    class Pyramid
	{
	public:
        Pyramid();
        Pyramid(int num_levels, const cv::Mat &color, const cv::Mat &depth);
        ~Pyramid();

        bool create(int num_levels, const cv::Mat &color, const cv::Mat &depth);

		cv::Mat color(int lvl = 0);
		cv::Mat intensity(int lvl = 0);
		cv::Mat depth(int lvl = 0);

	private:
		cv::Mat downsample(const cv::Mat &img);
		cv::Mat downsampleDepth(const cv::Mat &depth);
        std::vector<cv::Mat> createPyramid(int num_pyramid_levels, const cv::Mat &img);
        std::vector<cv::Mat> createDepthPyramid(int num_pyramid_levels, const cv::Mat &depth);

        std::vector<cv::Mat> color_pyramid_;
        std::vector<cv::Mat> intensity_pyramid_;
        std::vector<cv::Mat> depth_pyramid_;
	};

} 
