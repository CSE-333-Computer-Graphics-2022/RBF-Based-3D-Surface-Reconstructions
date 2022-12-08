

#pragma once

#include <nv/mat.h>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <nv/camera.h>


/**
 * @brief   Functionalities for processing the color images and depth maps
 *          contained in RGB-D frames
 * @author  Robert Maier <robert.maier@tum.de>
 */
namespace nv
{
    void threshold(cv::Mat &depth, float depth_min, float depth_max);

	cv::Mat computeVertexMap(const Mat3f &K, const cv::Mat &depth);

    cv::Mat computeNormals(const cv::Mat &vertex_map, float depth_threshold = 0.3f);
    cv::Mat computeNormals(const Mat3f &K, const cv::Mat &depth, float depth_threshold = 0.3f);

    cv::Mat resizeDepth(const Camera &input_cam, const cv::Mat &input_depth, const Camera &output_cam);

    cv::Mat erodeDiscontinuities(const cv::Mat &depth, int window_size, float max_depth_diff = 0.5f);

	template<typename T>
	T interpolate(const cv::Mat &img, float x, float y, int channel = 0);

	Vec3b interpolateRGB(const cv::Mat &color, float x, float y);

} 
