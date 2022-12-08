

#pragma once

#include <string>
#include <vector>
#include <nv/mat.h>
#include <nv/camera.h>
#include <nv/settings.h>
#include <opencv2/core.hpp>

namespace nv
{

    /**
     * @brief   Interface for accessing RGB-D sensors
     *          (contains already generic functionality)
     * @author  Robert Maier <robert.maier@tum.de>
     */
    class Sensor
	{
	public:
        virtual ~Sensor();

        static Sensor* create(const std::string &dataset);
        static Sensor* create(Settings &cfg);

        const Camera& depthCamera() const;
        Camera& depthCamera();
        const Camera& colorCamera() const;
        Camera& colorCamera();

		void setNumFramesMax(int n);
		int numFramesMax();
		int numFrames() const;
		bool frameExists(int id) const;

		void setDepthMin(float d);
		float depthMin() const;
		void setDepthMax(float d);
		float depthMax() const;

		cv::Mat depth(int id);
		cv::Mat color(int id);

		virtual void setPose(int id, const Mat4f &p) = 0;
		virtual Mat4f pose(int id) = 0;
		virtual double timePose(int id) = 0;
		virtual double timeDepth(int id) = 0;
		virtual double timeColor(int id) = 0;

		bool loadDepthIntrinsics(const std::string &filename);
        bool loadColorIntrinsics(const std::string &filename);

		static bool loadPoses(const std::string &filename, std::vector<Mat4f> &poses, 
                                std::vector<double> &timestamps, bool first_pose_is_identity = false);

		bool loadPoses(const std::string &filename);
		bool savePoses(const std::string &filename);

		void print() const;

	protected:
        static Sensor* create(const std::string &dataset, Settings &cfg);

        Sensor();
        Sensor(const Sensor&);
        Sensor& operator=(const Sensor&);

		virtual bool init(const std::string &dataset) = 0;

		void thresholdDepth(cv::Mat &depth) const;

		virtual cv::Mat loadDepth(int id) = 0;
		virtual cv::Mat loadColor(int id) = 0;

        Camera cam_depth_;
        Camera cam_color_;
        int num_frames_max_;
        int num_frames_;
        float depth_min_;
        float depth_max_;
	};

} 
