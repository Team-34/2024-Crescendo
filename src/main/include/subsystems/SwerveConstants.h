#pragma once

#include "Constants.h"

#include <array>
#include <cmath>
#include <limits>

#include <units/math.h>

#include <frc/filter/MedianFilter.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>


#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>


namespace t34 {

    template <typename T>
    constexpr T pow(T base, unsigned int exp) {
        return (exp >= sizeof(unsigned int)*8) ? 0 :
            exp == 0 ? 1 : base * pow(base, exp-1);
    }

    namespace Detail {
        constexpr double sqrtNewtonRaphson(double x, double curr, double prev) {
            return curr == prev ? curr : sqrtNewtonRaphson(x, 0.5 * (curr + x / curr), curr);
        }
    }

    /*
    * Constexpr version of the square root
    * Return value:
    *	- For a finite and non-negative value of "x", returns an approximation for the square root of "x"
    *   - Otherwise, returns NaN
    */
    constexpr double sqrt(double x) {
        return x >= 0 && x < std::numeric_limits<double>::infinity()
            ? Detail::sqrtNewtonRaphson(x, x, 0)
            : std::numeric_limits<double>::quiet_NaN();
    }




    // Left Forward Swerve Module
    constexpr int ID_LEFT_FWD_DRIVE     {20};    // Old IDs{ 22 };
    constexpr int ID_LEFT_FWD_STEER     {21};    // Old IDs{ 23 };
    constexpr int ID_LEFT_FWD_CANCODER  {22};    // Old IDs{ 2 };
    
    // Right Forward Swerve Module    
    constexpr int ID_RIGHT_FWD_DRIVE    {23};    // Old IDs{ 32 };
    constexpr int ID_RIGHT_FWD_STEER    {24};    // old IDs{ 33 };
    constexpr int ID_RIGHT_FWD_CANCODER {25};    // Old IDs{ 3 };
    
    // Right Aft Swerve Module    
    constexpr int ID_RIGHT_AFT_DRIVE    {26};    // Old IDs{ 52 };
    constexpr int ID_RIGHT_AFT_STEER    {27};    // Old IDs{ 53 };
    constexpr int ID_RIGHT_AFT_CANCODER {28};    // Old IDs{ 5 };
        
    // Left Aft Swerve Module    
    constexpr int ID_LEFT_AFT_DRIVE     {29};    // Old IDs{ 42 };
    constexpr int ID_LEFT_AFT_STEER     {30};    // Old IDs{ 43 };
    constexpr int ID_LEFT_AFT_CANCODER  {31};    // Old IDs{ 4 };

    constexpr double LEFT_FWD_CANCODER_OFFSET  { -0.006836 };
    constexpr double LEFT_AFT_CANCODER_OFFSET  { -0.607910 };
    constexpr double RIGHT_FWD_CANCODER_OFFSET { -0.899902 };
    constexpr double RIGHT_AFT_CANCODER_OFFSET { -0.440674 };

    constexpr double FRAME_LENGTH { 0.5715 / 2.0 }; // meters  (22.5 inches)
    constexpr double FRAME_WIDTH  { 0.5715 / 2.0 }; // meters  (22.5 inches)
    constexpr double SWERVE_MODULE_FROM_CENTER { sqrt((FRAME_LENGTH * FRAME_LENGTH) + (FRAME_WIDTH * FRAME_WIDTH)) };

    // SDS Mark 3, 4, and 4i Drive Gear Ratios 
    constexpr double SDSMK3_STANDARD { 8.16 / 1.0 }; 
    constexpr double SDSMK3_FAST    { 6.86 / 1.0 };
    constexpr double SDSMK4_L1       { 8.14 / 1.0 };
    constexpr double SDSMK4_L2       { 6.75 / 1.0 };
    constexpr double SDSMK4_L3       { 6.12 / 1.0 };
    constexpr double SDSMK4_L4       { 5.14 / 1.0 };
    constexpr double SDSMK4i_L1      { 8.14 / 1.0 };
    constexpr double SDSMK4i_L2      { 6.75 / 1.0 };
    constexpr double SDSMK4i_L3      { 6.12 / 1.0 };

    // TALONFX DRIVE CONFIG CONSTANTS
    constexpr bool    DRIVE_ENABLE_CURRENT_LIMIT        { true };
    constexpr int     DRIVE_CONTINUOUS_CURRENT_LIMIT    { 35 };
    constexpr int     DRIVE_PEAK_CURRENT_LIMIT          { 60 };
    constexpr double  DRIVE_PEAK_CURRENT_DURATION       { 0.1 };
    constexpr double  DRIVE_KP                          { 1.0 }; //0.5
    constexpr double  DRIVE_KI                          { 0.0 }; //3.0
    constexpr double  DRIVE_KD                          { 0.0 }; //0.0
    constexpr double  DRIVE_KF                          { 0.0 };
    constexpr double  DRIVE_KS                          { 0.32 / 12.0 };
    constexpr double  DRIVE_KV                          { 1.51 / 12.0 };
    constexpr double  DRIVE_KA                          { 0.27 / 12.0 };
    constexpr double  DRIVE_MAX_SPEED                   { 2.0 }; 
    constexpr double  DRIVE_TELEOP_MAX_SPEED            { 2.0 };
    constexpr double  DRIVE_GEAR_RATIO                  { SDSMK4_L1 };
    constexpr double  DRIVE_WHEEL_DIAMETER              { 4.0 };
    constexpr double  DRIVE_WHEEL_CIRCUMFERENCE         { DRIVE_WHEEL_DIAMETER * M_PIl };

    // TALONFX STEERING CONFIG CONSTANTS
    constexpr bool    STEER_ENABLE_CURRENT_LIMIT                { false };
    constexpr double  STEER_GEAR_RATIO                          { 12.8 / 1.0 };
    constexpr int     STEER_CONTINUOUS_CURRENT_LIMIT            { 25 };
    constexpr int     STEER_PEAK_CURRENT_LIMIT                  { 40 };
    constexpr double  STEER_PEAK_CURRENT_DURATION               { 0.1 };
    constexpr double  STEER_KP                                  { 60.0 };
    constexpr double  STEER_KI                                  { 0.0 };
    constexpr double  STEER_KD                                  { 0.05 };
    constexpr double  STEER_KF                                  { 0.0 };
    constexpr double  STEER_MAX_SPEED                           { 1.0 };
    constexpr double  TURNING_RADIUS_METERS                     { sqrt(pow(FRAME_LENGTH / 2.0, 2.0) + pow(FRAME_WIDTH / 2.0, 2.0)) };
    constexpr double  STEER_TELEOP_MAX_SPEED_RADIAN_PER_SECOND  { 1.5 / TURNING_RADIUS_METERS };

    constexpr double  FARIS_SPEED_MODE_SCALAR  { 0.2 }; // Pecent. Should be between 0.1 and 1.0
    constexpr double  ZERO_SWERVE_TIME_SECONDS { 5.0 };

}