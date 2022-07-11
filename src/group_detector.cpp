#include "predictive_proxemics_navigation/group_detector.h"


std::vector<Utils::GroupInformation> GroupDetector::detectGroups(const std::vector<Utils::PedestrianInformation>& t_pedestrians)
{
    std::vector<std::vector<Utils::PedestrianInformation>> groups{};

    for (const auto& person : t_pedestrians) {
        std::vector<Utils::PedestrianInformation> group{};
        group.emplace_back(person);
        groups.emplace_back(group);
    }

    std::vector<Point3f> group_centers{};
    for (int i = 0; i < groups.size(); ++i)
        group_centers.emplace_back(Point3f{0.0, 0.0, 0.0});

    bool converged{false};
    int n{0};
    while (!converged) {
        converged = true;

        for (int i = 0; i < groups.size(); ++i) {

            // calculate new center
            if (groups.at(i).size() > 1) {
                double avg_x{0.0};
                double avg_y{0.0};

                for (int j = 0; j < groups.at(i).size(); ++j) {
                    avg_x += groups.at(i).at(j).position.x;
                    avg_y += groups.at(i).at(j).position.y;
                }

                avg_x /= groups.at(i).size();
                avg_y /= groups.at(i).size();
                group_centers.at(i) = Point3f{avg_x, avg_y, 0.0};

            } else if (groups.at(i).size() == 1)
                group_centers.at(i) = Point3f{groups.at(i).at(0).position.x, groups.at(i).at(0).position.y, 0.0};
                
        }

        for (int i = 0; i < groups.size(); ++i) {
            for (int j = 0; j < groups.at(i).size(); ++j) {

                for (int k = 0; k < groups.size(); ++k) {
                    if (k != i) {
                        for (int l = 0; l < groups.at(k).size(); ++l) {

                            if (k < groups.size() && i < groups.size() && l < groups.at(k).size() && j < groups.at(i).size()) {

                                // FIXME: converge correctly
                                if (n == 2)
                                    converged = true;

                                double score_speed{scoreSpeed(groups.at(i).at(j).speed, groups.at(k).at(l).speed)};

                                if (score_speed < 0.0) {
                                    score_speed = 0.0;
                                }
                                
                                if (groups.at(i).at(j).speed == groups.at(k).at(l).speed)
                                    score_speed = 1.0;

                                double score{m_group_detection_score_distance_weight * scoreDistance(group_centers.at(i), groups.at(k).at(l).position) + m_group_detection_score_heading_weight * scoreHeading(groups.at(i).at(j).heading, groups.at(k).at(l).heading) + m_group_detection_score_speed_weight * score_speed};

                                if (score > m_group_detection_threshold) {                                
                                    converged = false;

                                    groups.at(i).emplace_back(groups.at(k).at(l));

                                    groups.at(k).erase(groups.at(k).begin() + l);
                                }
                            }
                        }

                    }
                }
            }
        }
        ++n;
    }

    // delete empty groups
    for (int i = 0; i < groups.size(); ++i) {
        if (groups.at(i).empty())
            groups.erase(groups.begin() + i);
    }

    // remove centers from deleted groups
    for (int i = 0; i < group_centers.size(); ++i) {
        for (int j = 0; j < groups.size(); ++j) {
            for (int k = 0; k < groups.at(j).size(); ++k) {
                if (i < group_centers.size() && j < groups.size() && k < groups.at(j).size()) {
                    if (group_centers.at(i).x == groups.at(j).at(k).position.x && group_centers.at(i).y == groups.at(j).at(k).position.y && groups.at(j).size() > 1)
                        group_centers.erase(group_centers.begin() + i);
                }
            }
        }
    }

    std::vector<Utils::GroupInformation> group_information{};
    for (const auto& group : groups) {
        if (!group.empty()) {
            double center_x{0.0};
            double center_y{0.0};
            double sum_direction_sin{0.0};
            double sum_direction_cos{0.0};
            double speed{0.0};

            for (const auto& person : group) {
                center_x += person.position.x;
                center_y += person.position.y;
                sum_direction_sin += std::sin(person.heading);
                sum_direction_cos += std::cos(person.heading);
                speed += person.speed;
            }

            center_x /= group.size();
            center_y /= group.size();

            double direction{std::atan2(sum_direction_sin, sum_direction_cos)};

            speed /= group.size();

            // calculate maximum distance
            double max_distance{0.0};
            for (const auto& person : group) {
                double euclidean_distance{Utils::euclideanDistance(person.position, Point3f{center_x, center_y, 0.0})};

                if (euclidean_distance > max_distance)
                    max_distance = euclidean_distance;
            }

            // TODO: look up in literature
            if (group.size() > 1)
                max_distance += 1.7;
            else
                max_distance += 1.2;

            group_information.emplace_back(Utils::GroupInformation{Point3f{center_x, center_y, 0.0}, direction, speed, max_distance});
        }
    }

    return group_information;
}


double GroupDetector::scoreDistance(const Point3f& t_position_1, const Point3f& t_position_2)
{
	// TODO: define externally
	return std::pow(M_E, -0.3 * Utils::euclideanDistance(t_position_1, t_position_2));	
}


double GroupDetector::scoreHeading(double heading_1, double heading_2)
{
	double a{Utils::angularDifferenceUnsigned(heading_1, heading_2)};

	double normalized{std::abs(a) / M_PI};

	if (normalized > 1.0)
		normalized = 1.0;

	return 1 - normalized;
}


double GroupDetector::scoreSpeed(double t_speed_1, double t_speed_2)
{
    double score{0.0};

    if (t_speed_1 == 0 && t_speed_2 == 0)
        score = 1.0;
    else {
        double difference{std::abs(t_speed_1 - t_speed_2)};

        if (t_speed_1 == 0) {
            score = 1 - std::abs(difference / t_speed_2);

            if (1 - std::abs(difference / t_speed_2) == -1)
                score = 0.0;

        } else {
            score = 1 - std::abs(difference / t_speed_1);

            if (1 - std::abs(difference / t_speed_1) == -1)
                score = 0.0;
        }
    }

    return score;
}
