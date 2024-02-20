#include <cmath>
#include "commands/ControllerDriveCommand.h"


namespace t34 {

    double ScaleToRange(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    inline bool IsInputZero(double x, double y, double r) {
        static double pos_tolerance =  0.00001;
        static double neg_tolerance = -0.00001;

        double combined = x + y + r;
        if (combined < pos_tolerance && combined > neg_tolerance) {
            return true;
        }

        return false;
    }

    ControllerDriveCommand::ControllerDriveCommand(std::shared_ptr<SwerveDrive> drive, std::shared_ptr<T34XboxController> controller)
        : m_swerve_drive(drive)
        , m_controller(controller) {
        
        AddRequirements(drive.get());

        m_last_zero = std::chrono::steady_clock::now();
    }

    void ControllerDriveCommand::Initialize() {
    
    }

    void ControllerDriveCommand::Execute() {
        double x    = m_controller->GetLeftStickXDB();
        double y    = m_controller->GetLeftStickYDB();
        double rot  = m_controller->GetRightStickXDB();

        if (IsInputZero(x, y, rot)) {
            m_swerve_drive->Stop();

            // When input there is no input and when the alloted time has elapsed, rezero the swerve wheels.
            // The alloted time is in seconds and can be set using the ZERO_SWERVE_TIME_SECONDS constexpr
            // located in SwerveContants.h. Suggested value is 5 seconds.
            std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - m_last_zero).count() > ZERO_SWERVE_TIME_SECONDS) {
                m_last_zero = now;
                m_swerve_drive->ResetToAbsolute();
            }  

            return;
        }

        double x_speed = std::copysign(ScaleToRange(-(x * x), 0.0, 1.0, 0.0, DRIVE_MAX_SPEED), x);
        double y_speed = std::copysign(ScaleToRange(-(y * y), 0.0, 1.0, 0.0, DRIVE_MAX_SPEED), y);
        double r_speed = std::copysign(ScaleToRange(-(rot * rot), 0.0, 1.0, 0.0, STEER_MAX_SPEED), rot);


        m_swerve_drive->Drive(frc::Translation2d{ units::meter_t(x_speed), units::meter_t(y_speed) }, r_speed);
    }

    void ControllerDriveCommand::End(bool interrupted) {
        m_swerve_drive->Stop();
    }

    bool ControllerDriveCommand::IsFinished() {
        return false;
    }

}