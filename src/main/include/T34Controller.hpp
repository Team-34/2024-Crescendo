#pragma once

#include <frc/XboxController.h>

namespace t34 {

    // Enumeration for specifying the target axis in a generic function
    enum class AxisType {
        Trigger,
        XAxis,
        YAxis,
        ZAxis
    };

    enum class JoystickHand {
    	Left,
    	Right,
    };

    // Subclass of the frc::XboxController adding extra functionality
    class T34XboxController : public frc::XboxController {
    public: // PUBLIC METHODS
        T34XboxController(int32_t port)
            : frc::XboxController(port)
            , m_left_x_db(0.0)
            , m_left_y_db(0.0)
            , m_right_x_db(0.0)
            , m_right_y_db(0.0)
            , m_left_trigger_db(0.0)
            , m_right_trigger_db(0.0) {

        }

        T34XboxController(T34XboxController &other)
            : frc::XboxController(other.GetPort())
            , m_left_x_db(0.0)
            , m_left_y_db(0.0)
            , m_right_x_db(0.0)
            , m_right_y_db(0.0)
            , m_left_trigger_db(0.0)
            , m_right_trigger_db(0.0) {
                
        }        

    	void SetAllAxisDeadband(double value) {
            if (value < 1.0 && value >= 0.0) {
                    m_left_x_db = value;
                    m_left_y_db = value;
                    m_right_x_db = value;
                    m_right_y_db = value;
                    m_left_trigger_db = value;
                    m_right_trigger_db = value;
            }
        }

        void SetAxisDeadband(JoystickHand hand, AxisType axis, double value) {
            // Clamp value from 0.0 to 1.0 
            if (value > 1.0) value = 1.0;
            if (value < 0.0) value = 0.0;

            switch(axis)
            {
                case AxisType::Trigger:
                    if (hand == JoystickHand::Left)
                        m_left_trigger_db = value;
                    else 
                        m_right_trigger_db = value;
                    break;
                case AxisType::XAxis:
                    if (hand == JoystickHand::Left)
                        m_left_x_db = value;
                    else
                        m_right_x_db = value;
                    break;
                case AxisType::YAxis:
                    if (hand == JoystickHand::Left)
                        m_left_y_db = value;
                    else    
                        m_right_y_db = value;
                default:
                    return;
            }
        }

        inline double GetXDB(JoystickHand hand) const  {
            double value = (hand == JoystickHand::Left ? GetLeftX() : GetRightX());
            double db = (hand == JoystickHand::Left ? m_left_x_db : m_right_x_db);

            if (value < 0.0) {
                if (value > -db) {
                    value = 0.0;
                }
            }
            else if (value < db) {
                value = 0.0;
            }

            return value;
        }

        inline double GetYDB(JoystickHand hand) const {
            double value = (hand == JoystickHand::Left ? GetLeftY() : GetRightY());
            double db = (hand == JoystickHand::Left ? m_left_y_db : m_right_y_db);

            if (value < 0.0) {
                if (value > -db) {
                    value = 0.0;
                }
            }
            else if (value < db) {
                value = 0.0;
            }

            return value;
        }

        inline double GetTriggerDB(JoystickHand hand) const {
            double value = (hand == JoystickHand::Left ? GetLeftTriggerAxis() : GetRightTriggerAxis());
            double db = (hand == JoystickHand::Left ? m_left_trigger_db : m_right_trigger_db);

            if (value < 0.0) {
                if (value > -db) {
                    value = 0.0;
                }
            }
            else if (value < db) {
                value = 0.0;
            }

            return value;
        }

    	inline double GetTriggersCoercedDB() const { return GetLeftTriggerDB() + -GetRightTriggerDB(); }
    	
    	inline double GetLeftStickXDB() const   { return GetXDB(JoystickHand::Left); }
    	inline double GetLeftStickYDB() const   { return GetYDB(JoystickHand::Left); }
    	inline double GetRightStickXDB() const  { return GetXDB(JoystickHand::Right); }
    	inline double GetRightStickYDB() const  { return GetYDB(JoystickHand::Right); }
    	inline double GetLeftTriggerDB() const  { return GetTriggerDB(JoystickHand::Left); }
    	inline double GetRightTriggerDB() const { return GetTriggerDB(JoystickHand::Right); }
    	
    private: //PRIVATE DATA
        double m_left_x_db;
        double m_left_y_db;
        double m_right_x_db;
        double m_right_y_db;
        double m_left_trigger_db;
        double m_right_trigger_db;
    };

}
