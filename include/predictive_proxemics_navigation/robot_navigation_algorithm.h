#pragma once

#include <map>
#include <vector>
#include <deque>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <ostream>

#include "pedestrian.h"
#include "utils.h"
#include "navigation_algorithm.h"
#include "group_detector.h"
#include "astar.h"


enum NAVIGATION_MODE
{
    NORMAL, EVASION, EVASION_DRIFTING, IN_SOCIAL_SPACE, COLLISON_AVOIDANCE
};

class RobotNavigationAlgorithm : public NavigationAlgorithm
{
public:
    RobotNavigationAlgorithm();
    RobotNavigationAlgorithm(float t_time_step, float t_max_speed, const std::vector<std::vector<int>> t_traversible, const Point3f& t_start, const Point3f& t_goal, const std::string& t_log_file_path);
    NAVIGATION_MODE m_navigation_mode = NORMAL;

    void noGlobalPlanner();

    void sense(const ros::Time& t_timestamp, const geometry_msgs::PoseStamped& t_robot_position, const std::vector<std::vector<geometry_msgs::PoseStamped>>& t_pedestrian_positions);
    void plan();
    geometry_msgs::PointStamped act();

    inline const bool mapTraversibleReadIn() const { return m_map_traversible_read_in; }

    AStar m_a_star{};

private:
    bool m_debug = false;
    float m_navigation_waypoint_met;
    float m_prediction_horizon;
    float m_proxemics_function_a_weight;
    float m_proxemics_function_sigma;
    float m_evasion_proxemics_data_points_step;
    float m_evasion_inner_score_intersection_weight;
    float m_evasion_inner_score_intrusion_weight;

    std::ofstream m_log_file{};

    double m_log_current_timestamp{};
    float m_log_current_robot_position_x{};
    float m_log_current_robot_position_y{};
    float m_log_current_distance_to_pedestrian{};

    Point3f m_start_position{};
    Point3f m_goal_position{};

    bool m_no_global_planner{false};

    bool m_inital_path_set = false;

    float m_sim_time;
    double m_evasion_distance{};

    std::vector<std::vector<int>> m_traversible;

    Point3f m_goal;

    float m_prediction_distance_threshold;

    std::deque<Point3f> m_waypoints;

    bool m_map_traversible_read_in = false;

    std::deque<geometry_msgs::PoseStamped> m_own_positions;
    std::vector<Utils::PedestrianInformation> m_pedestrian_positions_at_evasion;
    // std::vector<Utils::PedestrianInformation> m_detected_pedestrians{};
    GroupDetector m_group_detector;
    std::vector<std::pair<std::string, std::deque<geometry_msgs::PoseStamped>>> m_sensed_positions;
    float m_max_speed;
    float m_desired_speed;
    Vector3f m_velocity_vector{};
    //double m_distance_to_goal{MAXFLOAT};
    bool m_last_evasion_in_proxemic_area{false};
    Point3f m_last_evasion_position{};
    
    Point3f m_evasion_waypoint{};
    int m_last_evasion_id{-1};
    int m_number_of_waypoints{0};
    double m_current_speed{0.0};

    void applyGlobalPlanner();

    bool obstacleCollisionDetected(const Point3f& t_position);

    Utils::GroupInformation detectGroups();
    Vector3f drivingForce();

    double speedFunction(double t_distance);

    bool isInPersonalSpace(Vector3f t_robot_position, Vector3f t_pedestrian_position, double t_pedestrian_bearing_angle, double t_pedestrian_velocity, double t_min_distance);

    void evade(bool t_in_proxemic_area, Point3f t_predicted_robot_position, double t_robot_theta, Point3f t_predicted_person_position, double t_person_theta, double t_person_velocity, double t_min_distance);
    bool verifyEvasionWaypoint(int t_id, Point3f t_evasion_waypoint);

    // members used for logging
    int m_current_time{};
    Point3f m_current_robot_position{};
    double m_current_robot_heading{};
    Point3f m_current_pedestrian_position{};
    double m_current_pedestrian_heading{};
    std::vector<Point3f> m_current_pedestrian_prediction{};
    std::vector<Point3f> m_current_proxemic_boundary{};
    Point3f m_current_evasion_waypoint{};
    Point3f m_current_other_waypoint{};
};
