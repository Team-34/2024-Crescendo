#pragma once

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <units/math.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>

#include "Constants.h"

class Shooter
{
    public:

    Shooter();

    void runShooter(const double motor_output);

    void runIntake(const double motor_output);

    void moveToAngle(const double angle);

    void periodic();

    void init();

    rev::CANSparkMax m_firing_motorL;
    rev::CANSparkMax m_firing_motorR;
    rev::CANSparkMax m_arm_motorT;
    rev::CANSparkMax m_arm_motorB;

    rev::SparkMaxAbsoluteEncoder m_arm_encoderT;
    rev::SparkMaxAbsoluteEncoder m_arm_encoderB;

    ctre::phoenix::motorcontrol::can::TalonSRX m_intake;

    frc::PIDController m_arm_PID;

    frc::DigitalInput m_note_sensor;


    private:

    double m_arm_angleT{};
    double m_arm_angleB{};

};