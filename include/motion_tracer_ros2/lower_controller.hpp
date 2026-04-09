#ifndef LOWER_CONTROLLER_HPP_
#define LOEWR_CONTROLLER_HPP_

#include <iostream>
#include <math.h>
#include <fstream>  //iostream file

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class LowerController : public rclcpp::Node
{
public:
    LowerController();

    void init_follow_joint_trajectory();
    void sendJointAngles();
    void getJoy(const sensor_msgs::msg::Joy::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    trajectory_msgs::msg::JointTrajectory lifter_msg;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lifter_traj_pub_;

    std::map<std::string, double> joint_angles_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    geometry_msgs::msg::Twist cmd_vel_;

    int controller_rate_;     //[Hz]
    double controller_cycle_; //[sec]
    double move_time_;        //[sec]

    double rad2Deg = 180.0 / M_PI;
    double deg2Rad = M_PI / 180.0;

    // angle limit
    const float ankle_upper_limt = 1.57;
    const float ankle_lower_limt = 0;
    const float knee_upper_limt = 0;
    const float knee_lower_limt = -1.57;

    float lifter_ratio_;
};

#endif
