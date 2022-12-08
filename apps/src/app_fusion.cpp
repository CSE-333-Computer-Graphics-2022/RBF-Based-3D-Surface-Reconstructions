#include <iostream>

#include <opencv2/core.hpp>

#include <nv/mat.h>
#include <nv/app_fusion.h>
#include <nv/filesystem.h>
#include <nv/keyframe_selection.h>
#include <nv/mesh.h>
#include <nv/mesh/marching_cubes.h>
#include <nv/rgbd/sensor.h>
#include <nv/rgbd/processing.h>
#include <nv/sdf/algorithms.h>
#include <nv/sparse_voxel_grid.h>


namespace nv
{
    AppFusion::AppFusion() :
        sensor_(nullptr)
        {
        }

    AppFusion::~AppFusion()
	{
		delete sensor_;
	}
	
    bool AppFusion::run(int argc, char *argv[])
	{
        const char *keys = {
            "{s sensor| |sensor config file}"
            "{f fusion| |fusion config file}"
        };

		cv::CommandLineParser cmd(argc, argv, keys);
        
        std::string sensor_cfg_file = cmd.get<std::string>("sensor");
        std::string fusion_cfg_file = cmd.get<std::string>("fusion");

        Filesystem::changeWorkingDir(sensor_cfg_file, true);
        Filesystem::createFolder("./fusion");

        Settings sensor_cfg(sensor_cfg_file);
        sensor_ = Sensor::create(sensor_cfg);
        
        if (!sensor_)
            return false;
        
        sensor_->print();
        Settings fusion_cfg(fusion_cfg_file);
        
        if (!fuseSDF(fusion_cfg))
        {
            std::cerr << "SDF fusion failed!" << std::endl;
        
            return false;
        }

		cv::destroyAllWindows();

		return true;
	}


    bool AppFusion::fuseSDF(const Settings &fusion_cfg)
    {
        if (!sensor_ || fusion_cfg.empty())
            return false;

        const std::string keyframes_file = fusion_cfg.get<std::string>("keyframes");
        KeyframeSelection* keyframe_selection = nullptr;
        
        if (!keyframes_file.empty())
        {
            keyframe_selection = new KeyframeSelection();
        
            if (!keyframe_selection->load(keyframes_file))
                std::cerr << "Could not load keyframes ..." << std::endl;
        }

        SparseVoxelGrid<Voxel>* grid = SparseVoxelGrid<Voxel>::create(
            fusion_cfg.get<float>("voxel_size"), sensor_->depthMin(), sensor_->depthMax());
        
        if (!grid)
        {
            std::cerr << "Could not create voxel grid!" << std::endl;
        
            return false;
        }

        Vec6f clip_bounds;
        
        clip_bounds[0] = fusion_cfg.get<float>("clip_x0");
        clip_bounds[1] = fusion_cfg.get<float>("clip_x1");
        clip_bounds[2] = fusion_cfg.get<float>("clip_y0");
        clip_bounds[3] = fusion_cfg.get<float>("clip_y1");
        clip_bounds[4] = fusion_cfg.get<float>("clip_z0");
        clip_bounds[5] = fusion_cfg.get<float>("clip_z1");
        
        if (clip_bounds.norm() > 0.0f)
            grid->setClipBounds(clip_bounds);
        
        grid->printInfo();

        const int erode_size = fusion_cfg.get<int>("discont_window_size");
        std::cout << "Fusion..." << std::endl;
        
        for (int i = 0; i < sensor_->numFrames(); ++i)
        {
            if (keyframe_selection && !keyframe_selection->isKeyframe(i))
                continue;

            std::cout << "   integrating frame " << i << "... " << std::endl;
            cv::Mat depth = sensor_->depth(i);


            if (erode_size > 0)
                depth = erodeDiscontinuities(depth, erode_size);

            cv::Mat normals = computeNormals(sensor_->depthCamera().intrinsics(), depth);

            grid->integrate(sensor_->depthCamera(), sensor_->colorCamera(), depth, sensor_->color(i), normals, sensor_->pose(i));
        }

        std::cout << "correct SDF ..." << std::endl;
        SDFAlgorithms::correctSDF(grid);

        std::cout << "clear invalid voxels ..." << std::endl;
        SDFAlgorithms::clearInvalidVoxels(grid);

        std::cout << "Saving SDF (" << grid->numVoxels() << " voxels) ..." << std::endl;
        const std::string sdf_file = fusion_cfg.get<std::string>("output_sdf");
        
        if (!sdf_file.empty() && !grid->save(sdf_file))
            std::cerr << "Could not save SDF volume to file ..." << std::endl;

        std::cout << "Saving mesh ..." << std::endl;
        const std::string output_mesh_file = fusion_cfg.get<std::string>("output_mesh");
        
        if (!output_mesh_file.empty())
        {
            Mesh* mesh = MarchingCubes<Voxel>::extractSurface(*grid);
         
            if (!mesh)
                std::cerr << "Mesh could not be generated!" << std::endl;
            else if (!mesh->save(output_mesh_file))
                std::cerr << "Mesh could not be saved!" << std::endl;
            delete mesh;
        }

        delete grid;
        delete keyframe_selection;

        return true;
    }
}


int main(int argc, char *argv[])
{
    nv::AppFusion app;
	app.run(argc, argv);
    
    return 0;
}
