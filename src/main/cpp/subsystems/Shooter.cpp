#include "subsystems/Shooter.h"

t34::Shooter::Shooter()
: m_firing_motor_left(14, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_firing_motor_right(11, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_arm_motor_top(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
  m_arm_motor_bottom(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_intake_motor(12),
  m_firing_encoder_left(m_firing_motor_left.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42)),
  m_firing_encoder_right(m_firing_motor_right.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42)),
  m_arm_encoder_top(m_arm_motor_top.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42)),
  m_arm_encoder_bottom(m_arm_motor_bottom.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42)),
  m_note_sensor(9),
  m_arm_sensor(8),
  m_max_speed_percent(1.0),
  m_arm_angle_setpoint(90.0),
  m_tolerance(1.0),
  arm_using_pid(true),
  m_arm_pidctrl_top(m_arm_motor_top.GetPIDController()),
  m_arm_pidctrl_bottom(m_arm_motor_bottom.GetPIDController())
{
    m_arm_pidctrl_top.SetP(0.5);
    m_arm_pidctrl_top.SetI(0.0);
    m_arm_pidctrl_top.SetD(0.05);

    m_arm_pidctrl_bottom.SetP(0.5);
    m_arm_pidctrl_bottom.SetI(0.0);
    m_arm_pidctrl_bottom.SetD(0.05);
}

void t34::Shooter::RunShooterPercent(const double motor_output)
{
    const double speed = std::clamp(motor_output, -m_max_speed_percent, m_max_speed_percent);

    m_firing_motor_left.Set(speed);
    m_firing_motor_right.Set(speed);

    const bool is_firing = speed > 0.5;
    if (is_firing)
    {
        const auto left_speed = m_firing_encoder_left.GetVelocity();
        const auto right_speed = m_firing_encoder_right.GetVelocity();
        const auto avg_speed = (left_speed + right_speed) / 2;
    
        if (avg_speed >= speed)
        {
            const bool IGNORE_SENSOR = true;
            this->RunIntakeMotorPercent(speed, IGNORE_SENSOR);
        } 
    }
}

void t34::Shooter::RunTopArmMotorPercent(double motor_output)
{
    double bottom_clamp = m_arm_sensor.Get() ? 0.0 : -1.0;

    double top_clamp = ((GetTopArmEncoderVal() / ARM_DEG_SCALAR) > 90.0) ? 0.0 : 1.0;


    m_arm_motor_top.Set(std::clamp(motor_output, bottom_clamp, top_clamp));

}

void t34::Shooter::RunBottomArmMotorPercent(double motor_output)
{
    double bottom_clamp = m_arm_sensor.Get() ? 0.0 : -1.0;

    double top_clamp = ((GetBottomArmEncoderVal() / ARM_DEG_SCALAR) > 90.0) ? 0.0 : 1.0;

    m_arm_motor_bottom.Set(std::clamp(motor_output, bottom_clamp, top_clamp));
}

void t34::Shooter::SetZero()
{
    m_arm_encoder_top.SetPosition(0.0);
    m_arm_encoder_bottom.SetPosition(0.0);
}

void t34::Shooter::RunIntakeMotorPercent(const double motor_output, const bool bypass_sensor)
{
    double clamp_val = 1.0;

    if (IntakeHasNote() && (bypass_sensor == false))
    {
        clamp_val = 0.0;
    }

    m_intake_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                        std::clamp(motor_output, -1.0, clamp_val));


}

void t34::Shooter::ConfigForAmp()
{

    SetSetpoint(87.18);
    SetMaxSpeedPercent(0.15);
}

void t34::Shooter::ConfigForSpeaker(double shooter_firing_angle)
{

    //SetSetpoint(67.5);
    SetSetpoint(shooter_firing_angle);
    SetMaxSpeedPercent(1.0);
}

void t34::Shooter::ConfigForRest()
{
    SetSetpoint(90.0);
    SetMaxSpeedPercent(1.0);
    
}

void t34::Shooter::ConfigForNoteCollection()
{
    SetSetpoint(23.0);
    SetMaxSpeedPercent(1.0);
}

//void t34::Shooter::MoveArmToAngleDeg(const double angle) 
//{
//    m_arm_angle_setpoint = angle * ARM_DEG_SCALAR;
//
//    m_arm_pidctrl_top.SetReference(m_arm_angle_setpoint, rev::ControlType::kPosition);
//    m_arm_pidctrl_bottom.SetReference(m_arm_angle_setpoint, rev::ControlType::kPosition);
//    
//}
//
//void t34::Shooter::MoveShooterToAngleDeg(const double angle) 
//{
//    m_arm_angle_setpoint = ((angle - SHOOTER_OFFSET_ANGLE_DEG) * ARM_DEG_SCALAR);
//
//    MoveArmToAngleDeg(angle);
//}

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
    frc::SmartDashboard::PutNumber("Arm Setpoint: ", m_arm_angle_setpoint);

    //frc::SmartDashboard::PutBoolean("IsIntakeMovingBackwards: ", IsIntakeMovingBackward(m_intake_motor.GetMotorOutputPercent()));
    frc::SmartDashboard::PutBoolean("UsingPIDArmMovement: ", UsingPIDArmMovement());

}

void t34::Shooter::Periodic()
{
    m_arm_angle_setpoint = std::clamp(m_arm_angle_setpoint, 12.0, 90.0);

    //double motor_output = 
    //(fabs(m_kp * ( ( (m_arm_angle_setpoint / ARM_DEG_SCALAR) - GetTopArmEncoderVal()) / m_arm_angle_setpoint)) < m_tolerance) ? 
    //0.0 : (m_kp *  ( (m_arm_angle_setpoint / ARM_DEG_SCALAR) - GetTopArmEncoderVal()));

    if (UsingPIDArmMovement())
    {
        //m_arm_motor_top.Set(motor_output);
        //m_arm_motor_bottom.Set(motor_output);
        m_arm_pidctrl_top.SetReference((m_arm_angle_setpoint * ARM_DEG_SCALAR), rev::ControlType::kPosition);
        m_arm_pidctrl_bottom.SetReference((m_arm_angle_setpoint * ARM_DEG_SCALAR), rev::ControlType::kPosition);
    }
    
}

void t34::Shooter::Init()
{
    m_arm_encoder_top.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);
    m_arm_encoder_bottom.SetPositionConversionFactor(ARM_ENC_CONVERSION_FACTOR);

    m_arm_encoder_top.SetPosition(2.4803999);
    m_arm_encoder_bottom.SetPosition(m_arm_encoder_top.GetPosition());

    m_arm_motor_top.SetSmartCurrentLimit(20);
    m_arm_motor_bottom.SetSmartCurrentLimit(20);

    m_firing_motor_left.SetSmartCurrentLimit(30);
    m_firing_motor_right.SetSmartCurrentLimit(30);
}