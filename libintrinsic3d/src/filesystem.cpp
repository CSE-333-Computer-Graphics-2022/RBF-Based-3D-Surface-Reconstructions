

#include <nv/filesystem.h>

#include <iostream>
#include <algorithm>
#include <boost/filesystem.hpp>


namespace nv
{
namespace Filesystem
{

    bool changeWorkingDir(const std::string &path, bool verbose)
	{
		if (path.empty())
			return false;
		boost::filesystem::path p = boost::filesystem::system_complete(boost::filesystem::path(path));
		if (!boost::filesystem::exists(p))
			return false;
		if (!boost::filesystem::is_directory(p))
			p = p.branch_path();
		//std::cout << "path: " << p.string() << std::endl;
		boost::filesystem::current_path(p);

        if (verbose)
            std::cout << "Working dir: " << workingDir() << std::endl;

		return true;
	}


	std::string workingDir()
	{
		boost::filesystem::path p = boost::filesystem::current_path();
		return p.string();
	}


	bool createFolder(const std::string &path)
	{
		boost::filesystem::path p(path);
		if (!boost::filesystem::exists(p))
			boost::filesystem::create_directory(p);
		return boost::filesystem::exists(p);
	}

} // namespace Filesystem
} 
