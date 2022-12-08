

#pragma once


#include <vector>

#include <nv/mat.h>
#include <nv/settings.h>
#include <nv/sparse_voxel_grid.h>


namespace nv
{
	class KeyframeSelection;
	class Sensor;
	class Subvolumes;


    /**
     * @brief   Class for re-calculating the SDF colors based on desired
     *          function, which makes it possible to directly output hidden
     *          SDF properties (like voxel albedo or forward-shading) as
     *          mesh color
     * @author  Robert Maier <robert.maier@tum.de>
     */
	class SDFVisualization
	{
	public:

        /**
         * @brief   SDF visualization config struct
         * @author  Robert Maier <robert.maier@tum.de>
         */
        struct Config
        {
            const Subvolumes* subvolumes = nullptr;
            std::vector<Eigen::VectorXd> subvolume_sh_coeffs;
            bool largest_comp_only = false;
        };

        SDFVisualization(SparseVoxelGrid<VoxelSBR>* grid, const std::string &output_mesh_prefix);
		~SDFVisualization();

        static std::vector<std::string> getOutputModes(Settings &settings, bool add_voxel_colors = true);

        bool colorize(const std::vector<std::string> &color_modes, const SDFVisualization::Config &cfg);

	private:
        static void addOutputMode(Settings &settings, const std::string &param, const std::string &color_mode, std::vector<std::string> &color_modes);
        bool exportMesh(const std::string &output_mesh_prefix, const std::string &color_mode, bool largest_comp_only = false);

        bool storeColors();
        bool restoreColors();

        void applyColorNormals();
        void applyColorLaplacian();
        void applyColorIntensity();
        void applyColorIntensityGradient();
        void applyColorAlbedo();
        void applyColorShading(const Subvolumes* subvolumes,
                               const std::vector<Eigen::VectorXd> &sh_coeffs,
                               bool constant_albedo = false);
        void applyColorChromacity();
        void applyColorSubvolumes(const Subvolumes* subvolumes);
        void applyColorSubvolumesInterp(const Subvolumes* subvolumes);

        SparseVoxelGrid<VoxelSBR>* grid_;
        std::string output_mesh_prefix_;
		std::vector<Vec3b> colors_;

	};

} 
