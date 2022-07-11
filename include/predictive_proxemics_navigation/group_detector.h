#pragma once

#include <map>
#include <vector>
#include "utils.h"

class GroupDetector
{
public:
    GroupDetector() = default;

    std::vector<Utils::GroupInformation> detectGroups(const std::vector<Utils::PedestrianInformation>& t_pedestrians);

private:

    float m_group_detection_score_distance_weight = 0.74;
    float m_group_detection_score_heading_weight = 0.16;
    float m_group_detection_score_speed_weight = 0.1;

    // FIXME
    float m_group_detection_threshold = 0.74;

    double scoreDistance(const Point3f& t_position_1, const Point3f& t_position_2);

    double scoreHeading(double t_heading_1, double t_heading_2);

    double scoreSpeed(double t_speed_1, double t_speed_2);
};