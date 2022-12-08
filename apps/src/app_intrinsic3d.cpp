#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <nv/app_intrinsic3d.h>
#include <nv/filesystem.h>
#include <nv/keyframe_selection.h>
#include <nv/mesh.h>
#include <nv/mesh/marching_cubes.h>
#include <nv/rgbd/sensor.h>
#include <nv/refinement/intrinsic3d.h>
#include <nv/refinement/optimizer.h>
#include <nv/sdf/algorithms.h>
#include <nv/sdf/visualization.h>
#include <nv/lighting/lighting_svsh.h>
#include <nv/sparse_voxel_grid.h>


namespace nv
{
	AppIntrinsic3D::AppIntrinsic3D() :
        sensor_(nullptr),
        keyframe_selection_(nullptr),
        intrinsic3d_(nullptr)
	{
	}

	AppIntrinsic3D::~AppIntrinsic3D()
	{
		delete sensor_;
		delete keyframe_selection_;
		delete intrinsic3d_;
	}
	
	bool AppIntrinsic3D::run(int argc, char *argv[])
	{
        const char *keys = {
            "{s sensor| |sensor config file}"
            "{i intrinsic3d| |intrinsic3d config file}"
        };
	
    	cv::CommandLineParser cmd(argc, argv, keys);
        const std::string sensor_cfg_file = cmd.get<std::string>("sensor");
        const std::string intrinsic3d_cfg_file = cmd.get<std::string>("intrinsic3d");
        
        Filesystem::changeWorkingDir(sensor_cfg_file, true);       
        Filesystem::createFolder("./intrinsic3d");
        
        Settings sensor_cfg(sensor_cfg_file);
        sensor_ = Sensor::create(sensor_cfg);

        if (!sensor_)
            return false;
        
        sensor_->print();
        
        i3d_cfg_.load(intrinsic3d_cfg_file);

        if (i3d_cfg_.empty())
            return false;
		
        std::cout << "Loading Keyframes..." << std::endl;
		keyframe_selection_ = new KeyframeSelection();

        if (!keyframe_selection_->load(i3d_cfg_.get<std::string>("keyframes")))
        {
			std::cerr << "Could not load keyframes ..." << std::endl;
        }

        std::cout << keyframe_selection_->countKeyframes() << " keyframes loaded." << std::endl;	
		std::cout << "Loading SDF volume..." << std::endl;
        
        const std::string input_sdf_filename = i3d_cfg_.get<std::string>("input_sdf");
        SparseVoxelGrid<Voxel>* grid = SparseVoxelGrid<Voxel>::create(input_sdf_filename, sensor_->depthMin(), sensor_->depthMax());
		
        if (!grid)
		{
			std::cerr << "Could not load voxel grid!" << std::endl;
		
        	return false;
		}
        
        grid->printInfo();
        Intrinsic3D::Config i3d_cfg;
        i3d_cfg.load(i3d_cfg_);
        i3d_cfg.print();
        Optimizer::Config opt_cfg;
        opt_cfg.load(i3d_cfg_);
        opt_cfg.print();
        
        intrinsic3d_ = new Intrinsic3D(i3d_cfg, opt_cfg, sensor_, keyframe_selection_);	
		intrinsic3d_->addRefinementCallback(this);
		
		if (!intrinsic3d_->refine(grid))
		{
            std::cerr << "Intrinsic3D failed!" << std::endl;
			
            return false;
		}

		delete grid;	
		cv::destroyAllWindows();

		return true;
	}

    void AppIntrinsic3D::onSDFRefined(const Intrinsic3D::RefinementInfo &info)
	{
        std::string output_postfix = "_g" + std::to_string(info.grid_level) +
                                     "_p" + std::to_string(info.pyramid_level);
        std::string output_mesh_prefix = i3d_cfg_.get<std::string>("output_mesh_prefix");
        
        if (!output_mesh_prefix.empty())
		{	
            SparseVoxelGrid<VoxelSBR>* grid_vis = info.grid->clone();
            SDFAlgorithms::applyRefinedSdf(grid_vis);			
            std::vector<std::string> output_mesh_color_modes = SDFVisualization::getOutputModes(i3d_cfg_, true);            
            output_mesh_prefix = output_mesh_prefix + output_postfix;			
            SDFVisualization::Config config;
            config.subvolumes = &(intrinsic3d_->lighting()->subvolumes());
            config.subvolume_sh_coeffs = intrinsic3d_->lighting()->shCoeffs();
            config.largest_comp_only = i3d_cfg_.get<bool>("output_mesh_largest_comp_only");
            SDFVisualization sdf_vis(grid_vis, output_mesh_prefix);
            sdf_vis.colorize(output_mesh_color_modes, config);

            delete grid_vis;
		}

        std::string output_poses_prefix = i3d_cfg_.get<std::string>("output_poses_prefix");

        if (!output_poses_prefix.empty())
		{		
            std::string output_poses_file = output_poses_prefix + output_postfix + ".txt";
            std::cout << "Saving camera poses to file " << output_poses_file << std::endl;
         
            if (!sensor_->savePoses(output_poses_file))
				std::cerr << "Could not save poses..." << std::endl;
		}

        std::string output_intrinsics_prefix = i3d_cfg_.get<std::string>("output_intrinsics_prefix");

        if (!output_intrinsics_prefix.empty())
		{			
            std::string output_intrinsics_file = output_intrinsics_prefix + output_postfix + ".txt";
            std::cout << "Saving camera intrinsics to file " << output_intrinsics_file << std::endl;
         
            if (!sensor_->colorCamera().save(output_intrinsics_file))
				std::cerr << "Could not save color camera intrinsics!" << std::endl;
        }
	}
} 


int main(int argc, char *argv[])
{
	nv::AppIntrinsic3D app;
	app.run(argc, argv);
    
    return 0;
}
