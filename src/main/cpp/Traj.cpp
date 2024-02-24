#include "Traj.h"

double getAngle(const double velocity, const double distance, const double height)
{
    return 
    (
        (180 / 3.14159835359) *
        atan(
            (pow(velocity, 2) - 
            sqrt(
                pow(velocity, 4) - 9.807 * 
                (
                    (9.807 * pow(distance, 2)) + (2 * pow(velocity, 2) * height)
                )
            )) / (9.807 * distance)
        )
    );
}

bool isInRange(const double velocity, const double distance, const double angle)
{
    double rate = (
        tan(angle) - 
        (9.807 * distance) / (pow(angle, 2) * pow(velocity, 2))
    );

    if (rate > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

double getDistance(const double ll_angle, const double ll_height, const double target_height, const double target_v_offset)
{
    return ( 
        (target_height - ll_height) / tan(
            RAD_TO_DEG( (ll_angle + target_v_offset) ) 
        )
    );
}