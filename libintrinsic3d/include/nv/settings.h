

#pragma once


#include <map>
#include <string>
#include <opencv2/highgui.hpp>


namespace nv
{

    /**
     * @brief   Settings container for loading and saving parameters
     *          (no support for subgroups)
     * @author  Robert Maier <robert.maier@tum.de>
     */
	class Settings
	{
	public:
		Settings();
		Settings(const std::string &filename);
		~Settings();
		
		template<typename T>
		void set(const std::string &name, const T &val);
		template<typename T>
        T get(const std::string &name) const;

        bool empty() const;
        bool exists(const std::string &name) const;

		bool load(const std::string &filename);
        bool save(const std::string &filename);

	private:
		Settings(const Settings&);
        const Settings& operator=(const Settings&);

		void load(Settings &settings, cv::FileNode &fn);
        void save(Settings &settings, cv::FileStorage &fs);

        std::map<std::string, std::string> params_;
	};

} 
