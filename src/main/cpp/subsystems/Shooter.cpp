#include "subsystems/Shooter.h"

t34::Shooter::Shooter()
: m_firing_motor_left(14, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_firing_motor_right(11, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_arm_motor_top(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
  m_arm_motor_bottom(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_intake_motor(11),
  m_arm_encoder_top(m_arm_motor_top.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42)),
  m_arm_encoder_bottom(m_arm_motor_bottom.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42)),
  m_note_sensor(2),
  m_arm_sensor(1),
  m_max_speed_percent(1.0),
  arm_using_pid(false),
  m_arm_pidctrl_top(m_arm_motor_top.GetPIDController()),
  m_arm_pidctrl_bottom(m_arm_motor_bottom.GetPIDController())
  {
    m_arm_pidctrl_top.SetP(0.05);
    m_arm_pidctrl_top.SetI(0.0);
    m_arm_pidctrl_top.SetD(0.02);

    m_arm_pidctrl_bottom.SetP(0.05);
    m_arm_pidctrl_bottom.SetI(0.0);
    m_arm_pidctrl_bottom.SetD(0.02);
  }

void t34::Shooter::RunShooterPercent(const double motor_output)
{
    m_firing_motor_left.Set(std::clamp(motor_output, -m_max_speed_percent, m_max_speed_percent));
    m_firing_motor_right.Set(std::clamp(motor_output, -m_max_speed_percent, m_max_speed_percent));

    m_intake_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);
}

void t34::Shooter::SetZero()
{
    m_arm_encoder_top.SetPosition(0.0);
    m_arm_encoder_bottom.SetPosition(0.0);
}

void t34::Shooter::RunIntakeMotorPercent(const double motor_output)
{
    double clamp_val = -1.0;

    if (IntakeHasNote())
    {
        clamp_val = 0.0;
    }

    m_intake_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                        std::clamp(motor_output, clamp_val, 1.0));

    //m_intake_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, motor_output);
    //if (IntakeHasNote())// && IsIntakeMovingBackward(motor_output))
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
    m_arm_setpoint = angle * ARM_DEG_SCALAR;

    m_arm_pidctrl_top.SetReference(m_arm_setpoint, rev::ControlType::kPosition);
    m_arm_pidctrl_bottom.SetReference(m_arm_setpoint, rev::ControlType::kPosition);
    
}

void t34::Shooter::SetMaxSpeedPercent(const double percent)
{
    m_max_speed_percent = percent;
}


double t34::Shooter::GetMaxSpeedPercent() const
{
    return m_max_speed_percent;
}

void t34::Shooter::PutTelemetry()
{
    frc::SmartDashboard::PutNumber("Arm Top Relative Encoder: ", GetTopArmEncoderVal() / ARM_DEG_SCALAR);
    frc::SmartDashboard::PutNumber("Arm Bottom Relative Encoder: ", GetBottomArmEncoderVal() / ARM_DEG_SCALAR);
    
    frc::SmartDashboard::PutNumber("Max Speed: ", m_max_speed_percent);
    frc::SmartDashboard::PutNumber("Arm Setpoint: ", m_arm_setpoint);

    //frc::SmartDashboard::PutBoolean("IsIntakeMovingBackwards: ", IsIntakeMovingBackward(m_intake_motor.GetMotorOutputPercent()));
    frc::SmartDashboard::PutBoolean("UsingPIDArmMovement: ", UsingPIDArmMovement());

}

void t34::Shooter::Periodic()
{
    
}

void t34::Shooter::Init()
{
    m_arm_encoder_top.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);
    m_arm_encoder_bottom.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);

    m_arm_encoder_top.SetPosition(2.4803999);
    m_arm_encoder_bottom.SetPosition(2.4803999);
}