#pragma once

#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/controller/PIDController.h>

#include "Constants.h"

namespace t34
{

    class Climber
    {
        rev::CANSparkMax m_left_pulley;
        rev::CANSparkMax m_right_pulley;

        rev::SparkAbsoluteEncoder m_left_pulley_encoder; 
        rev::SparkAbsoluteEncoder m_right_pulley_encoder;

        frc::PIDController m_pulley_pid;

        bool m_locked_state;

        double m_left_pulley_inches;
        double m_right_pulley_inches;

    public:

        Climber();

        void Move(double inches_from_base);

        void Init();

        void Periodic();

        bool Lock();

        bool Unlock();

        inline bool IsLocked() { return m_locked_state; }

        inline void RunLeftPulley(const double motor_output) { m_left_pulley.Set(motor_output); }
        inline void RunRightPulley(const double motor_output) { m_right_pulley.Set(motor_output); }

        
        inline double GetLeftPulleyEncoderVal() { return m_left_pulley_encoder.GetPosition(); }
        inline double GetRightPulleyEncoderVal() { return m_right_pulley_encoder.GetPosition(); }


    };
}