#pragma once

#include <deque>
#include <vecmath.h>
#include "utils.hpp"


class Agent
{
public:
    Agent() = default;

    Agent(float t_speed, const std::deque<Point3f> t_waypoints);

    inline int id() const { return m_id; }

    inline Point3f& position() { return m_position; }
    
    inline const Point3f position() const { return m_position; }

    inline Vector3f& velocity() { return m_velocity; }

    inline const Vector3f velocity() const { return m_velocity; }

    inline float heading() const { return m_heading; }

    inline float& desiredSpeed() { return m_desired_speed; }

    inline const float desiredSpeed() const { return m_desired_speed; }

    inline std::deque<Point3f>& waypoints() { return m_path; }

    inline const std::deque<Point3f> waypoints() const { return m_path; }

    void addWaypoint(float t_x, float t_y);

    void addWaypointFront(float t_x, float t_y);

    void popWaypoint();

    Point3f path();

protected:
    int m_id;
    Point3f m_position;
    Vector3f m_velocity;
    float m_heading;
    float m_desired_speed;
    std::deque<Point3f> m_path;
};
