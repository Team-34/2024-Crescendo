#pragma once


constexpr bool INVERT_GYRO{ true }; // Always ensure Gyro is CCW+ CW-

constexpr double PI{ 3.1415926535897932384626433832 };
constexpr double PI2{ PI * 2.0 };
constexpr double _180_DIV_PI{ 180.0 / PI };
constexpr double PI_DIV_180{ PI / 180.0 };
#define DEG_TO_RAD(x) x * PI_DIV_180
#define RAD_TO_DEG(x) x * _180_DIV_PI




