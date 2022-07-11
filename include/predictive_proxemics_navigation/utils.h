#pragma once

#include <iostream>
#include <vecmath.h>
#include <vector>


namespace Utils
{
    struct Waypoint
    {
        Waypoint(float t_x, float t_y) : position{Point3f{t_x, t_y, 0.0}}, radius{0.5} {}
        Point3f position;
        float radius;
    };

    struct PedestrianInformation
    {
        int id;
        Point3f position;
        float heading;
        float speed;
        float yaw_rate{0.0};
        float acceleration{0.0};
        float distance;

        PedestrianInformation(int t_id, Point3f t_position, float t_heading, float t_speed, float t_distance) :
            id{t_id}, position{t_position}, heading{t_heading}, speed{t_speed}, distance{t_distance}
            {};

        PedestrianInformation(int t_id, Point3f t_position, float t_heading, float t_speed, float t_yaw_rate, float t_acceleration) :
            id{t_id}, position{t_position}, heading{t_heading}, speed{t_speed}, yaw_rate{t_yaw_rate}, acceleration{t_acceleration}
            {};
    };

    struct GroupInformation
    {  
        Point3f position;
        double theta;
        double speed;
        double yaw_rate;
        double acceleration;
        double max_distance;
        double distance;

        GroupInformation(const Point3f& t_position, double t_theta, double t_speed, double t_max_distance) :
            position{t_position}, theta{t_theta}, speed{t_speed}, max_distance{t_max_distance}
            {};
    };

    struct AgentState
    {
        std::string name;
        float x;
        float y;
        float theta;

        AgentState() = default;

        AgentState(const std::string& t_name, float t_x, float t_y, float t_theta) :
            name{t_name}, x{t_x}, y{t_y}, theta{t_theta}
            {};
    };

    double proxemicsFunction(double t_a, double t_x, double t_sigma, double t_d_min);

    float randomFloat(float t_min, float t_max);

    std::vector<std::vector<std::string>> readInPaths(const std::string& t_path);

    std::string readFileIntoString(const std::string& t_path);

    Point3f getNearestPointToWall(const std::pair<Point3f, Point3f>& t_wall, Point3f t_point);

    double circularMean(const std::vector<double>& t_angles);

    double angularDifferenceSigned(double t_angle_from, double t_angle_to);

    double angularDifferenceUnsigned(double t_angle_a, double t_angle_b);

    double normalizeAngle(double t_angle);

    double localAngle(const Point3f& t_from_position, const Point3f& t_to_position);

    Point3f lineLineIntersection(const Point3f& t_robot_position, const Point3f& t_robot_extrapolated_position, const Point3f& t_person_position, const Point3f& t_person_extrapolated_position);

    double euclideanDistance(const Point3f& t_point_a, const Point3f& t_point_b);

    std::vector<std::pair<Point3f, double>> createLaserScannerDataPoints(const Point3f& t_robot_position, double t_robot_heading, std::vector<std::pair<Point3f, Point3f>> t_obstacles);
}
