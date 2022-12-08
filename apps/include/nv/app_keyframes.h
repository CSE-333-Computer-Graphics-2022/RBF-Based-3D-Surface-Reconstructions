#pragma once

#include <nv/mat.h>
#include <nv/settings.h>


namespace nv
{
	class Sensor;

    class AppKeyframes
	{
        public:
            AppKeyframes();
            ~AppKeyframes();

            bool run(int argc, char *argv[]);

        private:
            bool selectKeyframes(const Settings &cfg);

            Sensor* sensor_;
	};

} 


int main(int argc, char *argv[]);