#pragma once

#include <string>
#include "LimelightHelpers.h"
#include <cmath>

const int CONTROLLER_PORT = 0;

constexpr bool INVERT_GYRO{ true }; // Always ensure Gyro is CCW+ CW-

//constexpr double PI1{ 3.1415926535897932384626433832 };

constexpr double PI2{ M_PI * 2.0 };
constexpr double _180_DIV_PI{ 180.0 / M_PI };
constexpr double PI_DIV_180{ M_PI / 180.0 };
constexpr double NEO550_RES{ 42 };
constexpr double INTAKE_GEAR_RATIO{ 203.636364 };
constexpr double ARM_ENC_CONVERSION_FACTOR{ 360.0 / (NEO550_RES * INTAKE_GEAR_RATIO) };
constexpr double CLIMBER_UNITS_TO_INCHES_FACTOR{ 1 };
constexpr double ARM_DEG_SCALAR{ 0.02756 };
constexpr double SHOOTER_DEG_SCALAR{ 0.0116 };
constexpr double LIMELIGHT_DEGREE_SCALAR{ 23.188 / 20.25 };//{ 21.1726 / 22.5 };
constexpr double SHOOTER_OFFSET_ANGLE_DEG{ 59 };

const std::string LIMELIGHT_TABLE_NAME{ "" };

const int POV_UP{ 0 };
const int POV_RIGHT{ 90 };
const int POV_DOWN{ 180 };
const int POV_LEFT{ 270 };

#define DEG_TO_RAD(x) (x * PI_DIV_180)
#define RAD_TO_DEG(x) (x * _180_DIV_PI)


// frc::SendableChooser<std::string> m_chooser;
// const std::string kAutoNameDefault = "Default";
// const std::string kAutoNameCustom = "My Auto";
// std::string m_autoSelected;


