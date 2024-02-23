#include "subsystems/Shooter.h"

Shooter::Shooter()
: m_firing_motorL(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_firing_motorR(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_arm_motorT(14, rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
  m_arm_motorB(5, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_intake(3),
  m_arm_PID(0.1, 0.0, 0.1),
  m_arm_encoderT(m_arm_motorT.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
  m_arm_encoderB(m_arm_motorB.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
  m_note_sensor(0)

  {}

void Shooter::runShooter(const double motor_output)
{
    m_firing_motorL.Set(-motor_output);
    m_firing_motorR.Set(-motor_output);

    runIntake(motor_output);
}

void Shooter::runIntake(const double motor_output)
{
    if (m_note_sensor.Get() && (-motor_output) < 0.0)
    {
        m_intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    }
    else
    {
        m_intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -motor_output);
    }
}

void Shooter::moveToAngle(const double angle) 
{
    m_arm_PID.SetSetpoint(angle);
    m_arm_motorT.Set(m_arm_PID.Calculate(m_arm_angleT));
    m_arm_motorB.Set(m_arm_PID.Calculate(m_arm_angleB));
}

void Shooter::periodic()
{
    
}

void Shooter::init()
{
    m_arm_encoderT.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);
    m_arm_encoderB.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);
}