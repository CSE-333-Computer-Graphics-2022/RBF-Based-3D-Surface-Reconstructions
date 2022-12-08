

#pragma once


#include <string>
#include <vector>
#include <nv/mat.h>
#include <nv/rgbd/sensor.h>
#include <opencv2/core.hpp>


namespace nv
{

    /**
     * @brief   RGB-D sensor for loading RGB-D data from
     *          Intrinsic3D dataset folder
     * @author  Robert Maier <robert.maier@tum.de>
     */
    class SensorI3d : public Sensor
	{
	public:
        SensorI3d();
        virtual ~SensorI3d();

		virtual void setPose(int id, const Mat4f &p);
		virtual Mat4f pose(int id);
		virtual double timePose(int id);
		virtual double timeDepth(int id);
		virtual double timeColor(int id);

	private:
		virtual bool init(const std::string &dataset);

        bool loadIntrinsics(const std::string &filename, Camera &cam) const;

        bool listFiles(std::vector<double> &timestamps_depth,
                       std::vector<std::string> &files_depth,
                       std::vector<double> &timestamps_color,
                       std::vector<std::string> &files_color,
                       std::vector<std::string> &files_poses) const;

		bool loadFile(const std::string &filename, std::vector<unsigned char> &data) const;
        bool loadFrame(const std::string &filename_depth, const std::string &filename_color,
                       std::vector<unsigned char> &depth, std::vector<unsigned char> &color) const;
        bool loadPose(const std::string &filename, Mat4f &pose) const;

		virtual cv::Mat loadDepth(int id);
		virtual cv::Mat loadColor(int id);

        std::string data_folder_;
        std::vector<Mat4f> poses_cam_to_world_;
        std::vector< std::vector<unsigned char> > depth_images_;
        std::vector< std::vector<unsigned char> > color_images_;
        std::vector<double> poses_timestamps_;
        std::vector<double> depth_timestamps_;
        std::vector<double> color_timestamps_;
	};

} 
