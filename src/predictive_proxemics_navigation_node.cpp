/*********************************************************************
* Software License Agreement (BSD 3-Clause License)
*
*  Copyright (c) 2019, Simon Janzon
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include <tf/transform_datatypes.h>
#include <memory>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <vecmath.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "predictive_proxemics_navigation/predictive_proxemics_navigation_node.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


PredictiveProxemicsNavigation::PredictiveProxemicsNavigation(const std::string& t_name)
        : m_emergency_stop{false}, m_status_module{}, m_tracking_module{}, m_parameter_nh{"~"}
{
    m_name = t_name;

    // subscribe to topics
    m_bumper_sub = m_nh.subscribe("/mobile_base/events/bumper", 10, &PredictiveProxemicsNavigation::bumperCallback, this);
    m_odom_sub = m_nh.subscribe("/odom", 10, &PredictiveProxemicsNavigation::odometryCallback, this);
    m_map_sub = m_nh.subscribe("/map", 10, &PredictiveProxemicsNavigation::mapCallback, this);
    m_pedestrian_sub = m_nh.subscribe("/path_smoothed", 1, &PredictiveProxemicsNavigation::pedestrianCallback, this);

    // advertise to topics
    m_velocity_command_pub = m_nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    m_visualization_pub = m_nh.advertise<visualization_msgs::Marker>("robust_people_follower/markers", 10);
    m_goal_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    // variables that hold launch parameter values
    std::string log_file_path{""};
    float robot_start_x{0.0};
    float robot_start_y{0.0};
    float robot_goal_x{0.0};
    float robot_goal_y{0.0};

    // read in parameters
    m_parameter_nh.param("logFilePath", log_file_path, log_file_path);
    m_parameter_nh.param("robotStartX", robot_start_x, robot_start_x);
    m_parameter_nh.param("robotStartY", robot_start_y, robot_start_y);
    m_parameter_nh.param("robotGoalX", robot_goal_x, robot_goal_x);
    m_parameter_nh.param("robotGoalY", robot_goal_y, robot_goal_y);

    // TODO: use traversible map
    std::vector<std::vector<int>> traversible{};

    // robot start and end point
    Point3f robot_start{robot_start_x, robot_start_y, 0.0};
    Point3f robot_goal{robot_goal_x, robot_goal_y, 0.0};

    m_navigation_algorithm = RobotNavigationAlgorithm{0.05, 1.2, traversible, robot_start, robot_goal, log_file_path};
}


PredictiveProxemicsNavigation::~PredictiveProxemicsNavigation()
{
    ROS_INFO_STREAM(m_name << " shutting down");

    m_bumper_sub.shutdown();
    m_odom_sub.shutdown();
    m_map_sub.shutdown();
    m_pedestrian_sub.shutdown();
    m_velocity_command_pub.shutdown();
    m_visualization_pub.shutdown();
}


void PredictiveProxemicsNavigation::runLoop()
{
    auto loop_rate{ros::Rate{LOOP_FREQUENCY}};

    // LOG
//    std::ofstream file{};
//    file.open("/home/simon/log.csv");
//    file << "t,x,y,d,y_d,v,m_v,m_m_v,th,m_th,m_m_th,p_x,p_y,p_v,d_t\n";
//    auto start_time{ros::Time::now().toSec()};

    bool flag{false};
    geometry_msgs::PointStamped last_waypoint{};

    while (ros::ok()) {

        // auto start{std::chrono::system_clock::now()};

        processCallbacks();

        m_navigation_algorithm.sense(ros::Time::now(), m_status_module.robotPose(), m_tracking_module.m_tracked_pedestrian_positions);
        m_navigation_algorithm.plan();
        auto waypoint{m_navigation_algorithm.act()};

        // convert waypoint to a stamped pose
        geometry_msgs::PoseStamped waypoint_to_publish{};
        waypoint_to_publish.header.frame_id = "map";
        waypoint_to_publish.header.stamp = ros::Time::now();
        waypoint_to_publish.pose.position.x = waypoint.point.x;
        waypoint_to_publish.pose.position.y = waypoint.point.y;
        waypoint_to_publish.pose.orientation.w = 1.0;
        
        // TODO: test
        m_velocity_command_pub.publish(m_control_module.velocityCommand(m_status_module.robotPose(), Point3f{waypoint.point.x, waypoint.point.y, waypoint.point.z}));

        // only publish new waypoints to prevent flooding
        // if (waypoint != last_waypoint) {
        //     m_goal_pub.publish(waypoint_to_publish);
        //     last_waypoint = waypoint;
        // }


        // TODO: publish markers in RViz

        loop_rate.sleep();
    }
}


void PredictiveProxemicsNavigation::processCallbacks()
{
    ros::spinOnce();

    // do an emergency stop if the flag is set
    if (m_emergency_stop) {

        // publish Twist message with values initialized to 0
        m_velocity_command_pub.publish(geometry_msgs::Twist{});

        m_emergency_stop = false;
    }

    // check if a target is lost
    // m_tracking_module.checkForTargetLoss(m_status_module.status());

    // reset flags to only process data once in a loop iteration
    m_status_module.valuesSet() = false;
    // for (auto& p : *m_tracking_module.trackedPersons())
        // p.valuesSet() = false;
}


void PredictiveProxemicsNavigation::debugPrintout() const
{
    ROS_INFO_STREAM("\n");

    ROS_INFO_STREAM("ROS time: " << ros::Time::now().sec);
    m_status_module.printInfo();
}

void PredictiveProxemicsNavigation::bumperCallback(const kobuki_msgs::BumperEventConstPtr& msg)
{
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
        m_emergency_stop = true;
}


void PredictiveProxemicsNavigation::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // save data from message
    auto pose_stamped{geometry_msgs::PoseStamped{}};
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = msg->pose.pose;

    // DEBUG
    std::cout << "received position: " << pose_stamped.pose.position.x << ", " << pose_stamped.pose.position.y << std::endl;

    // process data
    m_status_module.processOdometryData(pose_stamped);
}


// TODO: use smoothed path data
void PredictiveProxemicsNavigation::pedestrianCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    double ncloud = input->header.stamp.toSec();

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;

    pcl::fromROSMsg(*input, laserCloudIn);

    // std::vector<Point3f> pedestrian_positions{};

    // FIXME: split later on
    std::vector<geometry_msgs::PoseStamped> tracked_positions{};
    for (int i = 0; i < laserCloudIn.size(); i++) {

        geometry_msgs::PoseStamped current_pose{};
        current_pose.header.frame_id = "map";
        current_pose.header.stamp = ros::Time::now();
        current_pose.pose.position.x = laserCloudIn[i].x;
        current_pose.pose.position.y = laserCloudIn[i].y;

        tracked_positions.emplace_back(current_pose);
    }

    std::vector<std::vector<geometry_msgs::PoseStamped>> tracked_pedestrian_positions{};
    tracked_pedestrian_positions.emplace_back(tracked_positions);

    m_tracking_module.m_tracked_pedestrian_positions = tracked_pedestrian_positions;
}


void PredictiveProxemicsNavigation::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if (!m_map_set) {
        m_navigation_algorithm.m_a_star = AStar{msg};
        m_map_set = true;
    }
}


// TODO: reimplement
void PredictiveProxemicsNavigation::publishPersonMarkers() const
{
    // auto person_markers{std::vector<visualization_msgs::Marker>{}};
    // auto person_vectors{std::vector<visualization_msgs::Marker>{}};

    // auto i{0};
    // for (const auto& p : *m_tracking_module.trackedPersons()) {
    //     if (p.distance() > 0) {

    //         // person marker
    //         visualization_msgs::Marker marker{};
    //         marker.header.frame_id = "odom";
    //         marker.header.stamp = ros::Time::now();
    //         marker.ns = "persons";
    //         marker.id = i;
    //         marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    //         marker.action = visualization_msgs::Marker::ADD;
    //         marker.lifetime = ros::Duration{0.3};
    //         marker.pose.position.x = p.pose().pose.position.x;
    //         marker.pose.position.y = p.pose().pose.position.y;
    //         marker.pose.orientation.w = 1.0;

    //         marker.scale.x = 1.0;
    //         marker.scale.y = 1.0;
    //         marker.scale.z = 1.0;

    //         marker.color.a = 1.0;
    //         marker.color.r = 1.0;
    //         marker.color.g = 0.5;

    //         marker.mesh_resource = "package://robust_people_follower/meshes/standing.dae";

    //         // show a green marker for a tracked person
    //         if (p.target()) {
    //             marker.color.r = 0.0;
    //             marker.color.g = 1.0;
    //         }

    //         person_markers.emplace_back(marker);


    //         // person vector
    //         auto vector{visualization_msgs::Marker{}};
    //         vector.header.frame_id = "odom";
    //         vector.header.stamp = ros::Time::now();
    //         vector.ns = "vectors";
    //         vector.id = i;
    //         vector.type = visualization_msgs::Marker::ARROW;
    //         vector.action = visualization_msgs::Marker::ADD;
    //         vector.lifetime = ros::Duration{0.3};
    //         vector.pose.position.x = p.pose().pose.position.x;
    //         vector.pose.position.y = p.pose().pose.position.y;
    //         vector.pose.position.z = 1.3;

    //         auto q{tf::Quaternion{tf::createQuaternionFromYaw(p.meanAngle())}};
    //         vector.pose.orientation.x = q.getX();
    //         vector.pose.orientation.y = q.getY();
    //         vector.pose.orientation.z = q.getZ();
    //         vector.pose.orientation.w = q.getW();

    //         vector.scale.x = VECTOR_LENGTH_FACTOR * p.meanVelocity();
    //         vector.scale.y = 0.1;
    //         vector.scale.z = 0.1;

    //         vector.color.a = 1.0;
    //         vector.color.r = 1.0;

    //         person_vectors.emplace_back(vector);


    //         ++i;
    //     }
    // }

    // // publish markers
    // for (const auto& m : person_markers)
    //     m_visualization_pub.publish(m);
    // for (const auto& m : person_vectors)
    //     m_visualization_pub.publish(m);
}


void PredictiveProxemicsNavigation::publishWaypoints() const
{
    auto line_strip{visualization_msgs::Marker{}};
    auto line_list{visualization_msgs::Marker{}};

    line_strip.header.frame_id = line_list.header.frame_id = "odom";
    line_strip.ns = line_list.ns = "waypoints";
    line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    line_strip.id = 1;
    line_list.id = 2;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    // publish waypoints
    for (const auto& w : *m_control_module.waypoints()) {
        geometry_msgs::Point p{};
        p.x = w.point.x;
        p.y = w.point.y;

        line_strip.points.push_back(p);
        line_list.points.push_back(p);
        p.z += 1.0;
        line_list.points.push_back(p);
    }

    // publish markers
    m_visualization_pub.publish(line_strip);
    m_visualization_pub.publish(line_list);
}


/**
 * @brief Entry point for the node, runs the main loop.
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "predictive_proxemics_navigation");
    PredictiveProxemicsNavigation predictive_proxemics_navigation{ros::this_node::getName()};
    predictive_proxemics_navigation.runLoop();
    return 0;
}
