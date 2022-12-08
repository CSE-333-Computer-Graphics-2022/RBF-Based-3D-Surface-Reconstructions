#pragma once

#include <string>
#include <vector>

#include <nv/mat.h>
#include <nv/refinement/intrinsic3d.h>
#include <nv/settings.h>


namespace nv
{
	class KeyframeSelection;
    class Sensor;

    class AppIntrinsic3D : public Intrinsic3D::RefinementCallback
	{
        public:
            AppIntrinsic3D();
            virtual ~AppIntrinsic3D();

            bool run(int argc, char *argv[]);

            virtual void onSDFRefined(const Intrinsic3D::RefinementInfo &info);

        private:
            Settings i3d_cfg_;
            Sensor* sensor_;
            KeyframeSelection* keyframe_selection_;
            Intrinsic3D* intrinsic3d_;
	};

} 


int main(int argc, char *argv[]);