#include "subsystems/Climber.h"

// t34::Climber::Climber()
// :   m_left_pulley(99, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
//     m_right_pulley(98, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
//     m_left_pulley_encoder(m_left_pulley.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
//     m_right_pulley_encoder(m_right_pulley.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
//     m_pulley_pid(0.1, 0.0, 0.1),
//     m_left_pulley_inches(0.0),
//     m_right_pulley_inches(0.0),
//     m_locked_state(false) {}

// void t34::Climber::Move(double inches_from_base)
// {
//     m_pulley_pid.SetSetpoint(inches_from_base);


//     if (m_locked_state)
//     {
//         m_left_pulley.Set(0.0);
//         m_right_pulley.Set(0.0);
//     }
//     else
//     {
//         m_left_pulley.Set(m_pulley_pid.Calculate(m_left_pulley_inches));
//         m_right_pulley.Set(m_pulley_pid.Calculate(m_right_pulley_inches));
//     }
// }

// void t34::Climber::Init()
// {

// }

// void t34::Climber::Periodic()
// {
//     m_left_pulley_inches = GetLeftPulleyEncoderVal() * CLIMBER_UNITS_TO_INCHES_FACTOR;
//     m_right_pulley_inches = GetRightPulleyEncoderVal() * CLIMBER_UNITS_TO_INCHES_FACTOR;
// }

// bool t34::Climber::Lock()
// {
//     m_locked_state = true;

//     return m_locked_state;
// }

// bool t34::Climber::Unlock()
// {
//     m_locked_state = false;

//     return m_locked_state;
// }