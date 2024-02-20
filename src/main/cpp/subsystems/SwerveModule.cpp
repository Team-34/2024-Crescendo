#include <cmath>
#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/SwerveModule.h"

namespace t34 {

    static double RPMToMechanism(double rpm, double gear_ratio) {
        static double scalar = 2048.0 / 600.0;
        return (rpm * gear_ratio) * scalar; 
    }

    static double MechanismToRPM(double velocity_counts, double gear_ratio) {
        static double scalar = 600.0 / 2048.0;
        double rpm = velocity_counts * scalar;        
        return rpm / gear_ratio;
    }

    static double MechanismToMPS(double velocity_counts, double circumference, double gear_ratio) {
        double rpm = MechanismToRPM(velocity_counts, gear_ratio);
        return (rpm * circumference) / 60.0;
    }

    static double MPSToMechanism(double velocity, double circumference, double gear_ratio) {
        double rpm = (velocity * 60.0) / circumference;
        return RPMToMechanism(rpm, gear_ratio);
    }

    /**
     * Constructs a new SwerveModule object and initializes the
     * Drive & Steering Motors along with the CANcoder.
     * 
     * @param name A String representing the name of the module.
     *             Typically contains location of the module.
     * @param drive_id The can id of the drive motor. (Falcon 500)
     * @param steer_id The can id of the steer motor. (Falcon 500)
     * @param cancoder_id The can id of the CANcoder.
     */
    SwerveModule::SwerveModule(std::string name, const int drive_id, const int steer_id, const int cancoder_id)  
        : m_drive(new TalonFX(drive_id))
        , m_steer(new TalonFX(steer_id))
        , m_cancoder(new CANcoder(cancoder_id))
        , m_module_name(name) {

        // Get current configurations for all devices
        configs::TalonFXConfiguration steer_config{};
        m_steer->GetConfigurator().Refresh(steer_config);

        configs::TalonFXConfiguration drive_config{};
        m_drive->GetConfigurator().Refresh(drive_config);

        configs::CANcoderConfiguration cancoder_config{};
        m_cancoder->GetConfigurator().Refresh(cancoder_config);

        // Swerve CANCoder Configuration 
        cancoder_config.MagnetSensor.AbsoluteSensorRange = signals::AbsoluteSensorRangeValue::Unsigned_0To1;
        cancoder_config.MagnetSensor.SensorDirection = signals::SensorDirectionValue::Clockwise_Positive;

        m_cancoder->GetConfigurator().Apply(cancoder_config);


        // Setup steering configuration
        configs::TalonFXConfigurator& steer_configurator = m_steer->GetConfigurator();
        steer_config.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
        steer_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
        steer_config.CurrentLimits.SupplyCurrentLimitEnable = STEER_ENABLE_CURRENT_LIMIT;
        steer_config.CurrentLimits.SupplyCurrentLimit = STEER_CONTINUOUS_CURRENT_LIMIT;
        steer_config.CurrentLimits.SupplyCurrentThreshold = STEER_PEAK_CURRENT_LIMIT;
        steer_config.CurrentLimits.SupplyTimeThreshold = STEER_PEAK_CURRENT_DURATION;
        steer_config.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
        steer_config.Feedback.SensorToMechanismRatio = STEER_GEAR_RATIO;
        steer_config.Slot0.kP = STEER_KP;
        steer_config.Slot0.kI = STEER_KI;
        steer_config.Slot0.kD = STEER_KD;
        steer_config.HardwareLimitSwitch.ForwardLimitEnable = false;
        steer_config.HardwareLimitSwitch.ReverseLimitEnable = false;

        steer_configurator.Apply(steer_config);


        // Setup drive configuration 
        configs::TalonFXConfigurator& drive_configurator = m_drive->GetConfigurator();
        drive_config.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
        drive_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
        drive_config.CurrentLimits.SupplyCurrentLimitEnable = DRIVE_ENABLE_CURRENT_LIMIT;
        drive_config.CurrentLimits.SupplyCurrentLimit = DRIVE_CONTINUOUS_CURRENT_LIMIT;
        drive_config.CurrentLimits.SupplyCurrentThreshold = DRIVE_PEAK_CURRENT_LIMIT;
        drive_config.CurrentLimits.SupplyTimeThreshold = DRIVE_PEAK_CURRENT_DURATION;
        drive_config.Feedback.SensorToMechanismRatio = SDSMK4_L1;
        drive_config.Slot0.kP = DRIVE_KP;
        drive_config.Slot0.kI = DRIVE_KI;
        drive_config.Slot0.kD = DRIVE_KD;
        drive_config.HardwareLimitSwitch.ForwardLimitEnable = false;
        drive_config.HardwareLimitSwitch.ReverseLimitEnable = false;

        drive_configurator.SetPosition(units::angle::turn_t(0.0));
        
        drive_configurator.Apply(drive_config);

        m_last_angle = GetState().angle;

        ResetToAbsolute();        
    }

    /**
     * Convenience method to stop all output of both the drive
     * and steering motor. Typically called by 
     * SwerveDrive::Stop().
     */
    void SwerveModule::Stop() {
        m_drive->Set(0.0);
        m_steer->Set(0.0);
    }

    /**
     * Sets the desired speed (drive) and angle (steer) of the swerve module.
     * 
     * @param desired_state The desired state of the module.
     * @param is_open_loop Determines if the drive motor is set directly or 
     *                     by velocity.
     */
    void SwerveModule::SetDesiredState(frc::SwerveModuleState desired_state, bool is_open_loop) {
        // For steering, the swerve module should never have to change the angle of the wheel more than
        // 90 degrees to achieve the target angle.
        // Modify the desired_state's angle to be the shortest angle between the current and target angle.
        // Reverse direction of drive motor if necessary.
        desired_state = frc::SwerveModuleState::Optimize(desired_state, frc::Rotation2d(units::degree_t(m_steer->GetPosition().GetValue()/* * 360.0*/)));


        SetAngle(desired_state);
        SetSpeed(desired_state, is_open_loop);
    }

    /**
     * Sets the desired speed (drive) of the swerve module.
     * 
     * @param desired_state The desired state of the module.
     * @param is_open_loop Determines if the drive motor is set directly or 
     *                     by velocity.
     */
    void SwerveModule::SetSpeed(frc::SwerveModuleState desired_state, bool is_open_loop) {
        if(is_open_loop){
            double percent_output = desired_state.speed.value() / DRIVE_MAX_SPEED;
            m_drive->Set(percent_output);
        }
        else {
            double velocity = MPSToMechanism(desired_state.speed.value(), DRIVE_WHEEL_CIRCUMFERENCE, DRIVE_GEAR_RATIO);
            //double velocity = desired_state.speed.value() / DRIVE_WHEEL_CIRCUMFERENCE;

            if (velocity == 0) { 
                m_drive->SetVoltage(units::voltage::volt_t(0)); 
            }
            else {
                m_drive->SetControl(m_drive_velocity_voltage.WithVelocity(units::angular_velocity::turns_per_second_t(velocity)));
            }
        }
    }

    /**
     * Sets the desired angle (steer) of the swerve module.
     * 
     * @param desired_state The desired state of the module.
     */
    void SwerveModule::SetAngle(frc::SwerveModuleState desired_state) {
        //frc::Rotation2d angle = (fabs(desired_state.speed.value()) <= (DRIVE_MAX_SPEED * 0.005)) ? m_last_angle : desired_state.angle; 
//        m_steer->SetControl(m_steer_position_voltage.WithPosition(units::angle::turn_t(RadiansToRotations(angle.Radians().value()))));
//        m_last_angle = angle;


        m_steer->SetControl(m_steer_position_voltage.WithPosition(units::angle::turn_t(RadiansToRotations(desired_state.angle.Radians().value()))));
    } 

    /**
     * Sets the desired angle (steer) of the swerve module.
     * 
     * @param angle The desired state of the module.
     */
    void SwerveModule::SetAngle(frc::Rotation2d angle) {
        m_steer->SetControl(m_steer_position_voltage.WithPosition(units::angle::turn_t(angle.Radians().value())));
//        m_last_angle = frc::Rotation2d(units::radian_t(absAngle));
        m_last_angle = angle;
    }

    /**
     * Gets the current angle (steer) of the swerve module.
     * The angle returned is from the internal steer motor's
     * Rotor Sensor.
     * 
     * @return Rotation2d A rotation in a 2D coordinate frame.
     */
    frc::Rotation2d SwerveModule::GetAngle() {
        return frc::Rotation2d(units::degree_t(m_steer->GetPosition().GetValue() * 360.0));
    }

    /**
     * Gets the current angle (steer) of the swerve module to
     * be used for Odometry.
     * 
     * @return Rotation2d A rotation in a 2D coordinate frame.
     */
    frc::Rotation2d SwerveModule::GetAngleForOdometry() {
        return frc::Rotation2d(units::radian_t(RotationsToRadians(fmod(m_steer->GetPosition().GetValue().value(), 1.0))));
    }

    /**
     * Gets the current CANcoder angle (steer) of the swerve module.
     * 
     * @return Rotation2d A rotation in a 2D coordinate frame.
     */
    frc::Rotation2d SwerveModule::GetCanCoder() {
        return frc::Rotation2d(units::radian_t(RotationsToRadians(m_cancoder->GetAbsolutePosition().GetValue().value())));
    }

    /**
     * Sets the steers motor's rotor sensor to match the current
     * CANcoder value. (Typically called by SwerveDrive::ResetToAbsolute())
     */
    void SwerveModule::ResetToAbsolute() {
       m_steer->SetPosition(-(m_cancoder->GetAbsolutePosition().GetValue()));//RadiansToRotations(GetCanCoder().Radians().value())));
       //SetAngle(frc::Rotation2d(units::radian_t(0)));
    }

    /**
     * Gets the module's current state. (Speed and Angle)
     * 
     * @return SwerveModuleState Object representing the speed
     *         and angle of the swerve module.
     */
    frc::SwerveModuleState SwerveModule::GetState() {
        return  {
                    units::meters_per_second_t(MechanismToMPS(m_drive->GetPosition().GetValue().value(), DRIVE_WHEEL_CIRCUMFERENCE, DRIVE_GEAR_RATIO)), 
                    GetAngle()
                }; 
    }

    /**
     * Gets the module's current position.
     * 
     * @return SwerveModulePosition Object representing the distance
     *                              the drive module has traveled and
     *                              the angle of the swerve module.
     */
    frc::SwerveModulePosition SwerveModule::GetPosition() {
        return  { 
                    units::meter_t(m_drive->GetPosition().GetValue().value() * DRIVE_WHEEL_CIRCUMFERENCE), 
                    GetAngle()
                };
    }

    /**
     * Convenience method providing a location to output
     * telemetry to the SmartDashboard. This method is
     * called in SwerveDrive::PutTelemetry().
     */
    void SwerveModule::PutTelemetry() {
        frc::SmartDashboard::PutNumber(m_module_name + " CC Pos", m_cancoder->GetPosition().GetValue().value());
        frc::SmartDashboard::PutNumber(m_module_name + " Pos", m_steer->GetPosition().GetValue().value());
    }
}