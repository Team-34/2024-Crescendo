#pragma once

#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

class Climber
{
    public:

    Climber();

    void move(double inches_from_base);

    void init();

    void periodic();

    bool setLock(bool state);

    bool getIsLocked();

    rev::CANSparkMax m_left_pulley;
    rev::CANSparkMax m_right_pulley;

    rev::SparkAbsoluteEncoder m_left_pulley_encoder; 
    rev::SparkAbsoluteEncoder m_right_pulley_encoder;


    private:

    bool isLocked;

    double m_left_pulley_inches;

    
};