#pragma once
#include <string>


constexpr bool INVERT_GYRO{ true }; // Always ensure Gyro is CCW+ CW-

constexpr double PI{ 3.1415926535897932384626433832 };
constexpr double PI2{ PI * 2.0 };
constexpr double _180_DIV_PI{ 180.0 / PI };
constexpr double PI_DIV_180{ PI / 180.0 };
constexpr double NEO550_RES{ 42 };
constexpr double INTAKE_GEAR_RATIO{203.636364};
constexpr double ARM_ENC_CONVERSION_FACTOR{360.0 / (NEO550_RES * INTAKE_GEAR_RATIO)};
constexpr double ARM_DEG_SCALAR{ 0.02756 };
constexpr double SHOOTER_DEG_SCALAR{ 0.0116 };

const std::string LIMELIGHT_TABLE_NAME{ "" };

#define DEG_TO_RAD(x) (x * PI_DIV_180)
#define RAD_TO_DEG(x) (x * _180_DIV_PI)





