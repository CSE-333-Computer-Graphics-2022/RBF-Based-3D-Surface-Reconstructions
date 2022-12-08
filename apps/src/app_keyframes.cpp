#include <iostream>

#include <opencv2/core.hpp>

#include <nv/mat.h>
#include <nv/app_keyframes.h>
#include <nv/filesystem.h>
#include <nv/keyframe_selection.h>
#include <nv/rgbd/sensor.h>


namespace nv
{
    AppKeyframes::AppKeyframes() :
        sensor_(nullptr)
	{
	}

    AppKeyframes::~AppKeyframes()
	{
		delete sensor_;
	}

    bool AppKeyframes::run(int argc, char *argv[])
	{
        const char *keys = {
            "{s sensor| |sensor config file}"
            "{k keyframes| |keyframes selection config file}"
        };

		cv::CommandLineParser cmd(argc, argv, keys);

        const std::string sensor_cfg_file = cmd.get<std::string>("sensor");
        const std::string keyframes_cfg_file = cmd.get<std::string>("keyframes");

        Filesystem::changeWorkingDir(sensor_cfg_file, true);
        Filesystem::createFolder("./fusion");

        Settings sensor_cfg(sensor_cfg_file);
        sensor_ = Sensor::create(sensor_cfg);

		if (!sensor_)
			return false;

		sensor_->print();
        Settings keyframes_cfg(keyframes_cfg_file);

        if (!selectKeyframes(keyframes_cfg))
        {
            std::cerr << "Keyframe selection failed!" << std::endl;
            
            return false;
        }

		cv::destroyAllWindows();

		return true;
	}

    bool AppKeyframes::selectKeyframes(const Settings &cfg)
    {
        if (!sensor_ || cfg.empty())
            return false;

        const std::string keyframes_file = cfg.get<std::string>("filename");
        std::cout << "keyframes_file " << keyframes_file << std::endl;
        const int keyframe_selection_window = cfg.get<int>("window_size");
        std::cout << "keyframe_selection_window " << keyframe_selection_window << std::endl;

        if (keyframes_file.empty() || keyframe_selection_window == 0)
            return false;

        KeyframeSelection keyframe_selection(keyframe_selection_window);

        for (int i = 0; i < sensor_->numFrames(); ++i)
        {
            if (i % 50 == 0)
                std::cout << "Keyframe selection frame " << i << "... " << std::endl;

            keyframe_selection.add(sensor_->color(i));
        }

        keyframe_selection.selectKeyframes();
        keyframe_selection.save(keyframes_file);

        if (cfg.get<bool>("show_keyframes"))
        {
            for (int i = 0; i < sensor_->numFrames(); ++i)
            {
                if (!keyframe_selection.isKeyframe(i))
                    continue;
                
                cv::Mat color_score = sensor_->color(i).clone();
                keyframe_selection.drawScore(i, color_score);
                std::string window_name = "keyframe " + std::to_string(i);
                cv::imshow(window_name, color_score);
                cv::waitKey();
                cv::destroyWindow(window_name);
            }
        }

        return true;
    }
}


int main(int argc, char *argv[])
{
    nv::AppKeyframes app;
	app.run(argc, argv);
    
    return 0;
}
