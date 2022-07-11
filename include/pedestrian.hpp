#pragma once

#include "agent.hpp"
#include "utils.hpp"

class Pedestrian : public Agent
{
public:
    Pedestrian() = default;

    explicit Pedestrian(int t_id);

    Pedestrian(int t_id, const Point3f& t_position);
};
