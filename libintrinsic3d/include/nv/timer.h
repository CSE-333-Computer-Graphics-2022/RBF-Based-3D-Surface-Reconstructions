

#pragma once


#include <opencv2/core.hpp>


namespace nv
{

    /**
     * @brief   Timer class
     * @author  Robert Maier <robert.maier@tum.de>
     */
	class Timer
	{
	public:
		Timer() :
            start_time_(0.0),
			elapsed_(0.0)
		{
		}

		~Timer()
		{
		}

		void start()
		{
            start_time_ = static_cast<double>(cv::getTickCount());
		}

		void stop()
		{
			elapsed_ = elapsedSinceStart();
		}

		double elapsed() const
		{
			return elapsed_;
		}
	private:
		double elapsedSinceStart() const
		{
            return (static_cast<double>(cv::getTickCount()) - start_time_) / cv::getTickFrequency();
		}

        double start_time_;
		double elapsed_;
	};

} 
