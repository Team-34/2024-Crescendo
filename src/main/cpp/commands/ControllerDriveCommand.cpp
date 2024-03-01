#include <cmath>
#include "commands/ControllerDriveCommand.h"
#include <frc/filter/SlewRateLimiter.h>
#include <units/length.h>
#include <units/math.h>
#include <frc/XboxController.h> 
#include <frc/MathUtil.h>


namespace t34 {

   

    double ScaleToRange(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    inline bool IsInputZero(double x, double y, double rotation) {
        static double pos_tolerance =  0.00001;
        static double neg_tolerance = -0.00001;

        double combined = x + y + rotation;
        return combined < pos_tolerance && combined > neg_tolerance;
    }

    ControllerDriveCommand::ControllerDriveCommand(std::shared_ptr<SwerveDrive> drive, std::shared_ptr<T34XboxController> controller)
        : m_swerve_drive(drive)
        , m_controller(controller)
    {
        
        AddRequirements(drive.get());

        m_last_zero = std::chrono::steady_clock::now();
    }

    void ControllerDriveCommand::Initialize()
    {
    
    }

    void ControllerDriveCommand::Execute()
    {
        const double x        = m_controller->GetLeftStickXDB();
        const double y        = m_controller->GetLeftStickYDB();
        const double rotation = m_controller->GetRightStickXDB();

        if (IsInputZero(x, y, rotation)) {
            m_swerve_drive->Stop();

            // When there is no input and when the alloted time has elapsed, rezero the swerve wheels.
            // The alloted time is in seconds and can be set using the ZERO_SWERVE_TIME_SECONDS constexpr
            // located in SwerveContants.h. Suggested value is 5 seconds.
            std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - m_last_zero).count() > ZERO_SWERVE_TIME_SECONDS) {
                m_last_zero = now;
                m_swerve_drive->ResetToAbsolute();
            }  

            return;
        }

        const double x_speed        = std::copysign(ScaleToRange(-(x * x), 0.0, 1.0, 0.0, DRIVE_MAX_SPEED), x);
        const double y_speed        = std::copysign(ScaleToRange(-(y * y), 0.0, 1.0, 0.0, DRIVE_MAX_SPEED), y);
        const double rotation_speed = std::copysign(ScaleToRange(-(rotation * rotation), 0.0, 1.0, 0.0, STEER_MAX_SPEED), rotation);

        const auto limited_x_speed = m_x_speed_limiter.Calculate(frc::ApplyDeadband(x_speed, 0.02));

        const auto limited_y_speed = m_y_speed_limiter.Calculate(frc::ApplyDeadband(y_speed, 0.02));

        const auto limited_rotation_speed = m_rotation_limiter.Calculate(frc::ApplyDeadband(rotation_speed, 0.02));

        const auto drive_translation = frc::Translation2d {
            units::meter_t(limited_x_speed.to<double>()),
            units::meter_t(limited_y_speed.to<double>())
        };

        //units::scalar_t hello = frc::ApplyDeadband(rotation_speed, 0.02);


        // const double drive_rotation = rotation_speed;

        m_swerve_drive->Drive(
            drive_translation,
            limited_rotation_speed.to<double>()
        );
    }

    void ControllerDriveCommand::End(bool interrupted)
    {
        m_swerve_drive->Stop();
    }

    bool ControllerDriveCommand::IsFinished()
    {
        return false;
    }

}