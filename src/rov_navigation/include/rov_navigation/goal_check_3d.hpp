#pragma once

#include <cmath>

namespace rov_navigation
{

inline double yawFromQuaternion(double x, double y, double z, double w) 
{
    // Convert quaternion to yaw angle
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

inline double angleDifference(double angle1, double angle2) 
{
    double diff = angle1 - angle2;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

inline bool withinPositionTolerance(double dx, double dy, double dz, double xy_tolerance, double z_tolerance) 
{
    return (std::sqrt(dx * dx + dy * dy) <= xy_tolerance) && (std::abs(dz) <= z_tolerance);
}

inline bool withinYawTolerance(double current_yaw, double goal_yaw, double yaw_tolerance) 
{
    return std::abs(angleDifference(current_yaw, goal_yaw)) <= yaw_tolerance;
}

}