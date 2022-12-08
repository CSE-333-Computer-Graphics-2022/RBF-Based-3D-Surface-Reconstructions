

#pragma once


#include <nv/mat.h>
#include <string>


/**
 * @brief   Filesystem functions
 * @author  Robert Maier <robert.maier@tum.de>
 */
namespace nv
{
namespace Filesystem
{

    bool changeWorkingDir(const std::string &path, bool verbose = true);

	std::string workingDir();

	bool createFolder(const std::string &path);

} // namespace Filesystem
} 
