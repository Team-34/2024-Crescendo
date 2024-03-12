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

void t34::Shooter::MoveToAngleDeg(const double degrees) 
{
    const double setpoint_degrees = std::clamp(degrees, 0.0, 180.0);
    m_arm_pid.SetSetpoint(setpoint_degrees * ARM_DEG_SCALAR);
    
    m_arm_motor_top.Set(m_arm_pid.Calculate(GetTopArmEncoderVal()));
    m_arm_motor_bottom.Set(m_arm_pid.Calculate(GetBottomArmEncoderVal()));
    //m_arm_motor_top.Set(m_arm_pid.Calculate(m_arm_angle_top));
    //m_arm_motor_bottom.Set(m_arm_pid.Calculate(m_arm_angle_bottom));
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
    frc::SmartDashboard::PutNumber("Arm Setpoint: ", m_arm_pid.GetSetpoint() / ARM_DEG_SCALAR );

    frc::SmartDashboard::PutBoolean("IsIntakeMovingBackwards: ", IsIntakeMovingBackward(m_intake_motor.GetMotorOutputPercent()));
    frc::SmartDashboard::PutBoolean("UsingPIDArmMovement: ", UsingPIDArmMovement());

}

void t34::Shooter::RaiseArm()
{
    if (UsingPIDArmMovement())
    {
        m_arm_angle_setpoint_degrees += 1.0;
    }
    else
    {
        RunTopArmMotorPercent(0.4);
        RunBottomArmMotorPercent(0.4);
    }
}

void t34::Shooter::LowerArm()
{
    if (UsingPIDArmMovement())
    {
        m_arm_angle_setpoint_degrees -= 1.0;
    }
    else
    {
        RunTopArmMotorPercent(-0.4);
        RunBottomArmMotorPercent(-0.4);
    }
}

void t34::Shooter::StopArm()
{
    if (UsingPIDArmMovement())
    {
        // Nothing to do in PID mode
    }
    else
    {
        RunTopArmMotorPercent(0.0);
        RunBottomArmMotorPercent(0.0);
    }
}

void t34::Shooter::TargetAmp()
{
    // SetMaxSpeedPercent(0.1);
    SetMaxSpeedForAmp();
    m_arm_angle_setpoint_degrees = 87.18;
}

void t34::Shooter::TargetSpeaker(const double degrees)
{
    // SetMaxSpeedPercent(0.4);
    SetMaxSpeedForSpeaker();
    m_arm_angle_setpoint_degrees = degrees;
}

void t34::Shooter::TargetTrap()
{
    // SetMaxSpeedPercent(0.7);
    SetMaxSpeedForTrap();
    // m_arm_angle_setpoint_degrees = ???;
}

void t34::Shooter::CollectNotes()
{
    // SetMaxSpeedPercent(0.0);
    SetMaxSpeedForNoteCollection();
    m_arm_angle_setpoint_degrees = 10.0;
}

void t34::Shooter::RunIntake()
{
    RunIntakeMotorPercent(0.4);
}

void t34::Shooter::StopIntake()
{
    RunIntakeMotorPercent(0.0);
}

void t34::Shooter::TogglePIDArmMovement()
{
    m_arm_using_pid = !m_arm_using_pid;
    m_arm_angle_setpoint_degrees =
        ((GetTopArmEncoderVal() + GetBottomArmEncoderVal()) * 0.5) /
        ARM_DEG_SCALAR;
}

void t34::Shooter::Periodic()
{
    if (UsingPIDArmMovement())
    {
        MoveToAngleDeg(m_arm_angle_setpoint_degrees);
    }
    else
    {
        // Nothing to do when not in PID mode
    }
}

void t34::Shooter::Init()
{
    m_arm_encoder_top.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);
    m_arm_encoder_bottom.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);
}