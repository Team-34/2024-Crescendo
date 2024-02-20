#pragma once

#include <AHRS.h>

namespace t34 {

    class Gyro : public AHRS {
    public: // METHODS
        static Gyro* Get();

        void ZeroYaw();
        void ZeroHeading();
        void AutoGyro();
    

    private: // METHODS
        Gyro();
    };
    
}