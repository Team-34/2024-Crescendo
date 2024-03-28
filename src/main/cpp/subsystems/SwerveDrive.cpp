 #include <pathplanner/lib/auto/AutoBuilder.h>
 #include <pathplanner/lib/util/PathPlannerLogging.h>
 #include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
 #include <pathplanner/lib/util/PIDConstants.h>
 #include <pathplanner/lib/util/ReplanningConfig.h>

 #include <frc/smartdashboard/SmartDashboard.h>

#include <algorithm>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/SwerveDrive.h" 



using namespace pathplanner;

namespace t34 {

    /**
     * Constructs a new SwerveDrive. 
     */
    SwerveDrive::SwerveDrive() {
        SetName("SwerveDrive");
        m_gyro->Reset();

         // Configure the AutoBuilder last
         AutoBuilder::configureHolonomic(
             [this](){ return GetPose(); }, // Robot pose supplier
             [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
             [this]()
            { 
               //frc::ChassisSpeeds speeds{ GetRobotRelativeSpeeds() }; 
               //units::meters_per_second_t temp_vx{speeds.vx};
//
               //speeds.vx = -speeds.vy;
               //speeds.vy = temp_vx;
//
               //return speeds;
                return GetRobotRelativeSpeeds();
            }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
             [this](frc::ChassisSpeeds speeds)
            {
                //units::meters_per_second_t temp_vx{speeds.vx};
//
                //speeds.vx = -speeds.vy;
                //speeds.vy = temp_vx;

                DriveAuto(speeds); 
            }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
             HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                 PIDConstants(t34::DRIVE_KP, t34::DRIVE_KI, t34::DRIVE_KD), // Translation PID constants
                 PIDConstants(t34::STEER_KP, t34::STEER_KI, t34::STEER_KD), // Rotation PID constants
                 4.5_mps, // Max module speed, in m/s
                 0.4_m, // Drive base radius in meters. Distance from robot center to furthest module.
                 ReplanningConfig() // Default path replanning config. See the API for the options here
             ),
             []() {
                 // Boolean supplier that controls when the path will be mirrored for the red alliance
                 // This will flip the path being followed to the red side of the field.
                 // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                 auto alliance = frc::DriverStation::GetAlliance();
                 if (alliance) {
                     return alliance.value() == frc::DriverStation::Alliance::kRed;
                 }
                 return false;
             },
             this // Reference to this subsystem to set requirements
         );
    }

    /**
     * Toggles Faris speed mode on or off. When on, drive outputs
     * will be scaled by a percentage. This percentage is hard
     * coded into FARIS_SPEED_MODE_SCALAR constexpr located in
     * SwerveConstants.h and should be a value between 0.1 and 1.0. 
     */
    void SwerveDrive::ToggleFarisMode() {
        m_faris_mode = !m_faris_mode;
        if (m_faris_mode) {
            m_speed_scalar = std::clamp<double>(FARIS_SPEED_MODE_SCALAR, 0.1, 1.0);
        }
        else {
            m_speed_scalar = 1.0;
        }
    }

    /**
     * Computes the speed (velocity) and the angle for each 
     * swerve module for either for field relative or robot centric
     * mode. Then sends this data to each module which in-turn 
     * sets the appropriate motor outputs.
     * 
     * @param Translation2d A vector representing the x & y axis speeds.
     * @param rotation A value represent rotation speed.
     * @param field_relative true if field relative mode or false for 
     *                       robot centric mode.
     */
    void SwerveDrive::Drive(frc::Translation2d translation, double rotation, bool field_relative, bool is_open_loop) {
        
        m_field_oriented = field_relative;
        frc::ChassisSpeeds speeds;
        if (field_relative) {
            speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                    units::velocity::meters_per_second_t((translation.Y().value() / DRIVE_MAX_SPEED) * m_speed_scalar), 
                                    units::velocity::meters_per_second_t((translation.X().value() / DRIVE_MAX_SPEED )* m_speed_scalar), 
                                    units::radians_per_second_t(((rotation / STEER_MAX_SPEED) * m_speed_scalar)), 
                                    frc::Rotation2d(units::degree_t(m_gyro->GetYaw() * -1.0)) // SHOULD THIS BE INVERTED????
                                );
        } 
        else {
            // PROBABLY NEED TO SWAP X & Y: NOT YET TESTED
            speeds.vx = units::velocity::meters_per_second_t(-translation.X().value() / DRIVE_MAX_SPEED);
            speeds.vy = units::velocity::meters_per_second_t(translation.Y().value() / DRIVE_MAX_SPEED);
            speeds.omega = units::radians_per_second_t(rotation);
        }

        auto sms = m_swerve_drive_kinematics.ToSwerveModuleStates(speeds);

        frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&sms, units::meters_per_second_t(DRIVE_MAX_SPEED));

        for(size_t i = 0; i < m_swerve_modules.size(); ++i) {
            m_swerve_modules[i].SetDesiredState(sms[i], is_open_loop);
        }
    } 

    /**
     * Currently not implemented.
     */
    void SwerveDrive::DriveAuto(frc::ChassisSpeeds speeds) {

        //double x = units::convert<units::meters_per_second_t, units::meter_t> (speeds.vx.to<double>());

        //double y = units::convert<units::meters_per_second_t, units::meter_t> (speeds.vy.to<double>());

        //units::meter_t x = units::meter_t(speeds.vx.to<double>());
        //units::meter_t y = units::meter_t(speeds.vy.to<double>());
//
        //Drive(frc::Translation2d(units::meter_t(x), units::meter_t(y)), speeds.omega.value(), false, false);

        auto sms = m_swerve_drive_kinematics.ToSwerveModuleStates(speeds);

        frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&sms, units::meters_per_second_t(DRIVE_MAX_SPEED));

        for(size_t i = 0; i < m_swerve_modules.size(); ++i) {
            m_swerve_modules[i].SetDesiredState(sms[i], false);
        }


    }

    /**
     * Convenience method to stop all output of
     * both the drive and steering motor for all swerve
     * modules. 
     */
    void SwerveDrive::Stop() {
        for (auto& sm : m_swerve_modules) {
            sm.Stop();
        }
    }

    /**
     * Currently not implemented. Will be required for use
     * with PathPlanner.
     */
    frc::ChassisSpeeds SwerveDrive::GetRobotRelativeSpeeds() {
        //frc::ChassisSpeeds temp;

        return m_swerve_drive_kinematics.ToChassisSpeeds(GetModuleStates());
    }

    /**
     * Get the current pose of the robot.
     * 
     * @return Pose2d Object that contains translational and rotational elements.
     */
    frc::Pose2d SwerveDrive::GetPose() {
        return m_swerve_odometry.GetPose();
    }

    /**
     * Get the current pose's transitional x component of the robot.
     * 
     * @return double Represent the pose's transitional x component.
     */    
    double SwerveDrive::GetPoseX() {
        return m_swerve_odometry.GetPose().X().value();
    }

    /**
     * Get the current pose's transitional y component of the robot.
     * 
     * @return double Represent the pose's transitional y component.
     */    
    double SwerveDrive::GetPoseY() {
        return m_swerve_odometry.GetPose().Y().value();
    }

    /**
     * Get the current pose's rotational component of the robot.
     * 
     * @return double Represent the pose's rotational component in degrees.
     */
    double SwerveDrive::GetPoseRotation() {
        return m_swerve_odometry.GetPose().Rotation().Degrees().value();
    }

    /**
     * Reset the odometer for all swerve modules.
     */
    void SwerveDrive::ResetOdometry(frc::Pose2d pose) {
        m_swerve_odometry.ResetPosition(frc::Rotation2d(units::degree_t(m_gyro->GetAngle())), GetModulePositions(), pose);
    }  

    /**
     * Get an array of the swerve module states.
     * 
     * @return SwerveModuleState[4] Array of objects representing the speed
     *                              and angle of each swerve module.
     */
    std::array<frc::SwerveModuleState, 4> SwerveDrive::GetModuleStates() {
        std::array<frc::SwerveModuleState, 4> states;
        for (size_t i = 0; i < states.size(); ++i) {
            states[i] = m_swerve_modules[i].GetState();
        }

        return states;
    }

    /**
     * Get an array of the swerve positions.
     * 
     * @return SwerveModulePosition[4] Array of objects representing the distance
     *                                 traveled and the angle of each swerve module.
     */
    std::array<frc::SwerveModulePosition, 4> SwerveDrive::GetModulePositions() {
        std::array<frc::SwerveModulePosition, 4> positions = {
            frc::SwerveModulePosition{ m_swerve_modules[0].GetPosition().distance, m_swerve_modules[0].GetCanCoder() },
            frc::SwerveModulePosition{ m_swerve_modules[1].GetPosition().distance, m_swerve_modules[1].GetCanCoder() },
            frc::SwerveModulePosition{ m_swerve_modules[2].GetPosition().distance, m_swerve_modules[2].GetCanCoder() },
            frc::SwerveModulePosition{ m_swerve_modules[3].GetPosition().distance, m_swerve_modules[3].GetCanCoder() }
        };

        return positions;
    }

    /**
     * Get an array of the swerve positions.
     * 
     * @return SwerveModulePosition[4] Array of objects representing the inverted distance
     *                                 traveled and the angle of each swerve module.
     */
    std::array<frc::SwerveModulePosition, 4> SwerveDrive::GetModulePositionsInverted() {
        std::array<frc::SwerveModulePosition, 4> positions;
        for (size_t i = 0; i < positions.size(); ++i) {
            frc::SwerveModulePosition module_position = m_swerve_modules[i].GetPosition();
            positions[i] = { -(module_position.distance), module_position.angle };
        }

        return positions;
    }
    
    /**
     * Gets the yaw of the robot. A filter is used 
     * to lessen the impact of outliers in the gyro
     * data.
     * 
     * @return Rotation2d A rotation in a 2D coordinate frame.
     */    
    frc::Rotation2d SwerveDrive::GetYaw() {
        auto yaw = m_gyro->GetYaw();
        return INVERT_GYRO ? frc::Rotation2d(units::degree_t(m_filter.Calculate(360.0 - yaw))) : frc::Rotation2d(units::degree_t(m_filter.Calculate(yaw)));
    }

    /**
     * Sets the steers motor's rotor sensor to match the current
     * CANcoder value for each swerve module.
     */
    void SwerveDrive::ResetToAbsolute(){
        for(size_t i = 0; i < m_swerve_modules.size(); ++i) {
            m_swerve_modules[i].ResetToAbsolute();
        }
    }

    /**
     * DO NOT CALL DIRCTLY.
     * This method is called periodically by the CommandScheduler.
     * It is used here to update the swerve module odometer.
     */     
    void SwerveDrive::Periodic() {
        m_swerve_odometry.Update(GetYaw() * -1.0, 
        {
            frc::SwerveModulePosition{ m_swerve_modules[0].GetPosition().distance, m_swerve_modules[0].GetCanCoder() },
            frc::SwerveModulePosition{ m_swerve_modules[1].GetPosition().distance, m_swerve_modules[1].GetCanCoder() },
            frc::SwerveModulePosition{ m_swerve_modules[2].GetPosition().distance, m_swerve_modules[2].GetCanCoder() },
            frc::SwerveModulePosition{ m_swerve_modules[3].GetPosition().distance, m_swerve_modules[3].GetCanCoder() }
        });
        frc::SmartDashboard::PutData(this); 
    }

    /**
     * Convenience method providing a location to output
     * telemetry to the SmartDashboard. This method should
     * be called in Robot::RobotPeriodic().
     */
    void SwerveDrive::PutTelemetry() {
        frc::SmartDashboard::PutBoolean("Field Oriented", m_field_oriented);
        frc::SmartDashboard::PutBoolean("Faris Mode", m_faris_mode);

        for (auto& sm : m_swerve_modules) {
            sm.PutTelemetry();
        }
    }
}

