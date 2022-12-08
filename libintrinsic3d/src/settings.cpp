

#include <nv/settings.h>

#include <iostream>
#include <sstream>


namespace nv
{

	Settings::Settings()
	{
	}


	Settings::Settings(const std::string &filename)
	{
		if (!filename.empty())
			if (!load(filename))
				std::cerr << "Failed to load settings from file '" << filename << "'!" << std::endl;
	}


	Settings::Settings(const Settings&)
	{
	}


	Settings::~Settings()
	{
	}


    const Settings& Settings::operator=(const Settings& s)
	{
        return s;
	}
	

	template<typename T>
	void Settings::set(const std::string &name, const T &val)
	{
		std::stringstream ss;
		ss << val;
		params_[name] = ss.str();
	}
	template void Settings::set(const std::string &name, const bool &val);
	template void Settings::set(const std::string &name, const char &val);
	template void Settings::set(const std::string &name, const unsigned char &val);
	template void Settings::set(const std::string &name, const short &val);
	template void Settings::set(const std::string &name, const unsigned short &val);
	template void Settings::set(const std::string &name, const int &val);
	template void Settings::set(const std::string &name, const unsigned int &val);
	template void Settings::set(const std::string &name, const long long &val);
	template void Settings::set(const std::string &name, const unsigned long long &val);
	template void Settings::set(const std::string &name, const float &val);
	template void Settings::set(const std::string &name, const double &val);
	template void Settings::set(const std::string &name, const std::string &val);

	template<typename T>
    T Settings::get(const std::string &name) const
	{
		std::string val;
        const auto it = params_.find(name);
        if (it == params_.end())
		{
			std::cerr << "warning: settings parameter '" << name << "' not found!" << std::endl;
			// TODO handle other non-numeric types
			if (std::is_same<T, std::string>::value)
				val = "";
			else
				val = "0";
		}
		else
		{
            val = it->second;
		}

		std::stringstream ss(val);
		T result;
		ss >> result;
		return result;
	}
    template char Settings::get(const std::string &name) const;
    template bool Settings::get(const std::string &name) const;
    template unsigned char Settings::get(const std::string &name) const;
    template short Settings::get(const std::string &name) const;
    template unsigned short Settings::get(const std::string &name) const;
    template int Settings::get(const std::string &name) const;
    template unsigned int Settings::get(const std::string &name) const;
    template long long Settings::get(const std::string &name) const;
    template unsigned long long Settings::get(const std::string &name) const;
    template size_t Settings::get(const std::string &name) const;
    template float Settings::get(const std::string &name) const;
    template double Settings::get(const std::string &name) const;
    template std::string Settings::get(const std::string &name) const;


    bool Settings::empty() const
	{
        return params_.empty();
	}


    bool Settings::exists(const std::string &name) const
	{
        return params_.find(name) != params_.end();
	}


	bool Settings::load(const std::string &filename)
	{
		if (filename.empty())
			return false;
		// open file
		cv::FileStorage fs;
		try
		{
			fs.open(filename, cv::FileStorage::READ);
		}
		catch (...)
		{
			return false;
		}
		if (!fs.isOpened())
			return false;
        // load parameters recursively
        cv::FileNode fs_root = fs.root();
        load(*this, fs_root);
		// close file
		fs.release();
		return true;
	}


    bool Settings::save(const std::string &filename)
	{
		if (filename.empty())
			return false;
		// open file
		cv::FileStorage fs(filename, cv::FileStorage::WRITE);
		if (!fs.isOpened())
			return false;
        // save parameters recursively
		save(*this, fs);
		// close file
		fs.release();
		return true;
	}
	

	void Settings::load(Settings &settings, cv::FileNode &fn)
	{
		// iterate through nodes
		for (cv::FileNodeIterator itr = fn.begin(); itr != fn.end(); ++itr)
		{
			cv::FileNode n = *itr;
			std::string name = n.name();
            if (!n.isMap())
            {
				// load param
                settings.params_[name] = std::string(fn[name]);
			}
		}
	}


    void Settings::save(Settings &settings, cv::FileStorage &fs)
	{
		// save params
		for (auto itr : settings.params_)
			fs << itr.first << itr.second;
	}

} 
