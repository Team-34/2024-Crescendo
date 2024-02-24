#include "subsystems/Shooter.h"

t34::Shooter::Shooter()
: m_firing_motor_left(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_firing_motor_right(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_arm_motor_top(14, rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
  m_arm_motor_bottom(5, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_intake_motor(3),
  m_arm_pid(0.1, 0.0, 0.1),
  m_arm_encoder_top(m_arm_motor_top.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
  m_arm_encoder_bottom(m_arm_motor_bottom.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
  m_note_sensor(0)

  {}

void t34::Shooter::RunShooter(const double motor_output)
{
    m_firing_motor_left.Set(-motor_output);
    m_firing_motor_right.Set(-motor_output);

    RunIntake(motor_output);
}

void t34::Shooter::RunIntake(const double motor_output)
{
    if (IntakeHasNote() && IsIntakeMovingBackward(motor_output))
    {
        m_intake_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    }
    else
    {
        m_intake_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -motor_output);
    }
}

void t34::Shooter::MoveToAngle(const double angle) 
{
    m_arm_pid.SetSetpoint(angle);
    
    m_arm_motor_top.Set(m_arm_pid.Calculate(m_arm_angle_top));
    m_arm_motor_bottom.Set(m_arm_pid.Calculate(m_arm_angle_bottom));
}

void t34::Shooter::Periodic()
{
    
}

void t34::Shooter::Init()
{
    m_arm_encoder_top.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);
    m_arm_encoder_bottom.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);
}