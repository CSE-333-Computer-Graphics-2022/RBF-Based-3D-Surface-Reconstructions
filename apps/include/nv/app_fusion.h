#pragma once

#include <nv/mat.h>
#include <nv/settings.h>


namespace nv
{
	class Sensor;

    class AppFusion
	{
        public:
            AppFusion();
            ~AppFusion();

            bool run(int argc, char *argv[]);

        private:
            bool fuseSDF(const Settings &cfg);

            Sensor* sensor_;
	};

}


int main(int argc, char *argv[]);