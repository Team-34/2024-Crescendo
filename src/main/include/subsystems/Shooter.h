#pragma once

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <units/math.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>

#include "Constants.h"

namespace t34
{
    class Shooter
    {
        rev::CANSparkMax m_firing_motor_left;
        rev::CANSparkMax m_firing_motor_right;
        rev::CANSparkMax m_arm_motor_top;
        rev::CANSparkMax m_arm_motor_bottom;

        rev::SparkMaxAbsoluteEncoder m_arm_encoder_top;
        rev::SparkMaxAbsoluteEncoder m_arm_encoder_bottom;

        ctre::phoenix::motorcontrol::can::TalonSRX m_intake_motor;

        frc::PIDController m_arm_pid;

        frc::DigitalInput m_note_sensor;

        double m_arm_angle_top{};
        double m_arm_angle_bottom{};

        inline bool IntakeHasNote() { return m_note_sensor.Get(); }
        inline bool IsIntakeMovingBackward(const double motor_output) { return (-motor_output) < 0.0; }

    public:

        Shooter();

        void RunShooter(const double motor_output);

        void RunIntake(const double motor_output);

        void MoveToAngle(const double angle);

        void Periodic();

        void Init();

        inline double GetTopArmEncoderVal() { return m_arm_encoder_top.GetPosition(); }
        inline double GetBottomArmEncoderVal() { return m_arm_encoder_top.GetPosition(); }

        inline void RunTopArmMotor(const double motor_output) { m_arm_motor_top.Set(motor_output); }
        inline void RunBottomArmMotor(const double motor_output) { m_arm_motor_bottom.Set(motor_output); }

        inline void RunLeftFiringMotor(const double motor_output) { m_firing_motor_left.Set(motor_output); }
        inline void RunRightFiringMotor(const double motor_output) { m_firing_motor_right.Set(motor_output); }

        
        inline void RunIntakeMotor(const double motor_output) { m_intake_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, motor_output); }

        
    };
}