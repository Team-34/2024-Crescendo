#include "Gyro.h"

#include "Constants.h"

namespace t34 {

    static std::unique_ptr<Gyro> g_gyro{ nullptr };

    Gyro* Gyro::Get() {
        if (!g_gyro) {
            g_gyro.reset(new Gyro());
        }

        return g_gyro.get();
    }

    Gyro::Gyro()
        : AHRS(frc::SPI::Port::kMXP) {

    }


    void Gyro::ZeroYaw() { Reset(); }
    void Gyro::ZeroHeading() { ZeroYaw(); }
    //void Gyro::AutoGyro() { g_gyro->SetYaw(180.0); }

}