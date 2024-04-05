#pragma once

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <units/math.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SwerveDrive.h"

#include "Constants.h"

namespace t34
{
    class Shooter
    {
        rev::CANSparkMax m_firing_motor_left;
        rev::CANSparkMax m_firing_motor_right;
        rev::CANSparkMax m_arm_motor_top;
        rev::CANSparkMax m_arm_motor_bottom;

        rev::SparkMaxRelativeEncoder m_arm_encoder_top;
        rev::SparkMaxRelativeEncoder m_arm_encoder_bottom;

        rev::SparkMaxPIDController m_arm_pidctrl_top;
        rev::SparkMaxPIDController m_arm_pidctrl_bottom;

        ctre::phoenix::motorcontrol::can::TalonSRX m_intake_motor;

        //frc::PIDController m_arm_pid;

        frc::DigitalInput m_note_sensor{9};
        frc::DigitalInput m_arm_sensor{8};



        //double m_arm_angle_top{};
        //double m_arm_angle_bottom{};
        double m_max_speed_percent{};
        double m_arm_angle_setpoint{};
        double m_tolerance{};
        double m_kp{};

        bool arm_using_pid{};
        
        inline bool IsIntakeMovingBackward(const double motor_output) { return (-motor_output) < 0.0; }

        

    public:
        Shooter();

        void RunShooterPercent(const double motor_output);

        void RunIntakeMotorPercent(const double motor_output, const bool bypass_sensor = false);

        void MoveShooterToAngleDeg(const double angle);
        void MoveArmToAngleDeg(const double angle);

        void SetMaxSpeedPercent(const double percent);
        void ConfigForAmp();
        void ConfigForSpeaker(double shooter_firing_angle);
        void ConfigForRest();
        void ConfigForNoteCollection();

        double GetMaxSpeedPercent() const;

        void Periodic();

        void Init();

        void PutTelemetry();

        void SetZero();

        inline void SetSetpoint(double setpoint) {m_arm_angle_setpoint = setpoint;}

        inline void MoveUp() { m_arm_angle_setpoint += 0.5; }
        inline void MoveDown() { m_arm_angle_setpoint -= 0.5; }

        inline double GetTopArmEncoderVal() const { return m_arm_encoder_top.GetPosition(); }
        inline double GetBottomArmEncoderVal() const { return m_arm_encoder_bottom.GetPosition(); }

        void RunTopArmMotorPercent(double motor_output);// { m_arm_motor_top.Set(motor_output); }
        void RunBottomArmMotorPercent(double motor_output);// { m_arm_motor_bottom.Set(motor_output); }

        inline void RunLeftFiringMotorPercent(const double motor_output) { m_firing_motor_left.Set(motor_output); }
        inline void RunRightFiringMotorPercent(const double motor_output) { m_firing_motor_right.Set(motor_output); }

        inline void TogglePIDArmMovement() { arm_using_pid = !arm_using_pid; }
        inline bool UsingPIDArmMovement() const { return arm_using_pid; }

        inline bool IntakeHasNote() { return m_note_sensor.Get(); }
        inline bool IsArmAtZero() { return m_arm_sensor.Get(); }

        inline void SetTolerance(const double t) {m_tolerance = t; }
        inline void SetkP(const double kP) {m_kp = kP; }
    };
}