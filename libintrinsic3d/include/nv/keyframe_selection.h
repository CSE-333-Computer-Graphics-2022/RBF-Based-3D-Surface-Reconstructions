

#pragma once

#include <nv/mat.h>
#include <string>
#include <opencv2/core.hpp>

namespace nv
{

    /**
     * @brief   Keyframe selection class
     *          Selects a keyframe from a window of input color images based
     *          on the image blurriness (Crete et al)
     * @author  Robert Maier <robert.maier@tum.de>
     */
	class KeyframeSelection
	{
	public:
        KeyframeSelection(int window_size = 10);
		~KeyframeSelection();

		bool load(const std::string &filename);
		bool save(const std::string &filename) const;

		void reset();
		void add(const cv::Mat &color);
		void selectKeyframes();

        bool isKeyframe(int id) const;

        size_t countKeyframes() const;

        void drawScore(int id, cv::Mat &image) const;

	private:
		bool frameExists(int id) const;

        double estimateBlur(const cv::Mat &color) const;
        double estimateBlurCrete(const cv::Mat &gray) const;

        int window_size_;
        std::vector<double> frame_scores_;
        std::vector<bool> is_keyframe_;
	};

} 
