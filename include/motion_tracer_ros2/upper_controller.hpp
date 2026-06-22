#ifndef UPPER_CONTROLLER_HPP_
#define UPPER_CONTROLLER_HPP_

#include <iostream>
#include <cmath>
#include <map>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "aero_controller_msgs/srv/run_script.hpp"

class UpperController : public rclcpp::Node
{
public:
    UpperController();
    ~UpperController();

    void init_follow_joint_trajectory();
    void graspControl(std::string position, std::string _pose);
    void tracerStateCallback(const sensor_msgs::msg::JointState& _tracer_data);
    void sendJointAngles();
    void getJoy(const sensor_msgs::msg::Joy::SharedPtr msg);
    double calcHandAngle(double _position, double offset);

private:
    std::mutex msg_mtx;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr tracer_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    trajectory_msgs::msg::JointTrajectory rarm_msg;
    trajectory_msgs::msg::JointTrajectory larm_msg;
    trajectory_msgs::msg::JointTrajectory waist_msg;
    trajectory_msgs::msg::JointTrajectory head_msg;
    trajectory_msgs::msg::JointTrajectory rhand_msg;
    trajectory_msgs::msg::JointTrajectory lhand_msg;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr rarm_traj_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr larm_traj_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr waist_traj_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr head_traj_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr rhand_traj_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr lhand_traj_pub_;

    rclcpp::Client<aero_controller_msgs::srv::RunScript>::SharedPtr grasp_client_;

    std::map<std::string, double> joint_angles_;

    int controller_rate_;     // [Hz]
    double controller_cycle_; // [sec]
    double move_time_;        // [sec]

    const double rad2Deg = 180.0 / M_PI;
    const double deg2Rad = M_PI / 180.0;

    std::string r_hand_state;
    std::string l_hand_state;

    // angle limit
    const float neck_p_upper_limt = 1.0;
    const float neck_p_lower_limt = -0.3;
    const float neck_r_upper_limt = 0.087;
    const float neck_r_lower_limt = -0.087;
    const float neck_y_upper_limt = 1.57;
    const float neck_y_lower_limt = -1.57;
    const float waist_p_upper_limt = 0.68;
    const float waist_p_lower_limt = -0.16;
    const float waist_r_upper_limt = 0.12;
    const float waist_r_lower_limt = -0.12;
    const float waist_y_upper_limt = 2.09;
    const float waist_y_lower_limt = -2.09;

    bool send_angle_r_hand;
    bool send_angle_r_arm;
    bool send_angle_l_hand;
    bool send_angle_l_arm;

    //////////////////////////////
    // GUI parameters
    std::string neck_movement_;
    int neck_offset_;
    bool neck_auto_;
    bool neck_reverse_;
    //////////////////////////////
};

#endif