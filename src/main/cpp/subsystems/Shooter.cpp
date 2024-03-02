#include "subsystems/Shooter.h"

t34::Shooter::Shooter()
: m_firing_motor_left(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_firing_motor_right(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_arm_motor_top(14, rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
  m_arm_motor_bottom(11, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_intake_motor(11),
  m_arm_pid(0.5, 0.0, 0.25),
  m_arm_encoder_top(m_arm_motor_top.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42)),
  m_arm_encoder_bottom(m_arm_motor_bottom.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42)),
  m_note_sensor(1),
  //m_arm_angle_top(0.0),
  //m_arm_angle_bottom(0.0),
  m_max_speed_percent(1.0)

  {}

void t34::Shooter::RunShooterPercent(const double motor_output)
{
    m_firing_motor_left.Set(std::clamp(motor_output, -m_max_speed_percent, m_max_speed_percent));
    m_firing_motor_right.Set(std::clamp(motor_output, -m_max_speed_percent, m_max_speed_percent));
}

void t34::Shooter::RunIntakeMotorPercent(const double motor_output)
{

    m_intake_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, motor_output);
    //if (IntakeHasNote() && IsIntakeMovingBackward(motor_output))
    //{
    //    m_intake_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    //}
    //else
    //{
    //    m_intake_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -motor_output);
    //}
}

void t34::Shooter::MoveToAngleDeg(const double angle) 
{
    m_arm_pid.SetSetpoint(angle * ARM_DEG_SCALAR);
    
    m_arm_motor_top.Set(m_arm_pid.Calculate(GetTopArmEncoderVal()));
    m_arm_motor_bottom.Set(m_arm_pid.Calculate(GetTopArmEncoderVal()));
    //m_arm_motor_top.Set(m_arm_pid.Calculate(m_arm_angle_top));
    //m_arm_motor_bottom.Set(m_arm_pid.Calculate(m_arm_angle_bottom));
}

void t34::Shooter::SetMaxSpeedPercent(const double percent)
{
    m_max_speed_percent = percent;
}

void t34::Shooter::PutTelemetry()
{
    frc::SmartDashboard::PutNumber("Arm Top Relative Encoder: ", GetTopArmEncoderVal() / ARM_DEG_SCALAR);
    frc::SmartDashboard::PutNumber("Arm Bottom Relative Encoder: ", GetBottomArmEncoderVal() / ARM_DEG_SCALAR);
    
    frc::SmartDashboard::PutNumber("Max Speed: ", m_max_speed_percent);
    frc::SmartDashboard::PutNumber("Arm Setpoint: ", m_arm_pid.GetSetpoint() / ARM_DEG_SCALAR );

    frc::SmartDashboard::PutBoolean("IsIntakeMovingBackwards: ", IsIntakeMovingBackward(m_intake_motor.GetMotorOutputPercent()));
    frc::SmartDashboard::PutBoolean("UsingPIDArmMovement: ", UsingPIDArmMovement());

}

void t34::Shooter::Periodic()
{
    
}

void t34::Shooter::Init()
{
    m_arm_encoder_top.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);
    m_arm_encoder_bottom.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);
}