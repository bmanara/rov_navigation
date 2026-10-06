#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>


namespace rov_navigation
{

struct Vec3D 
{
    double x {0.0};
    double y {0.0};
    double z {0.0};
};

inline double distance(const Vec3D& a, const Vec3D& b) 
{
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2) + std::pow(b.z - a.z, 2));
}

inline std::vector<Vec3D> generateStraightLinePath3D(const Vec3D& start, const Vec3D& goal, double resolution) 
{
    if (resolution <= 0.0) 
    {
        throw std::invalid_argument("Resolution must be positive.");
    }

    std::vector<Vec3D> path;
    double total_distance = distance(start, goal);
    int num_points = static_cast<int>(std::ceil(total_distance / resolution));

    for (int i = 0; i <= num_points; ++i) 
    {
        double t = static_cast<double>(i) / num_points;
        Vec3D point;
        point.x = start.x + t * (goal.x - start.x);
        point.y = start.y + t * (goal.y - start.y);
        point.z = start.z + t * (goal.z - start.z);
        path.push_back(point);
    }

    return path;
}

} // namespace rov_navigation