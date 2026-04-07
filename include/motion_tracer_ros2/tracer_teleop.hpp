#ifndef TRACER_TELEOP_HPP_
#define TRACER_TELEOP_HPP_

#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "motion_tracer_ros2/tracer_command.hpp"

class TracerTeleop : public rclcpp::Node
{
public:
    TracerTeleop();
    ~TracerTeleop();

    void processLoop();

    void processPacket(const std::vector<uint8_t>& packet);

private:
    tracer::controller::TracerCommand *tracer_;
    std::vector<uint8_t> tracer_data_;
    std::vector<int16_t> position_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tracer_state_pub_;
    sensor_msgs::msg::JointState tracer_state_;

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    sensor_msgs::msg::Joy joy_;

    // dummy cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    geometry_msgs::msg::Twist cmd_vel_;

    bool wheel_stop_flag_;

    bool tracer_mode_;
    int init_counter_;

    int16_t pre_r_wrist_y_, pre_l_wrist_y_;

    bool enable_joy_;

    //////////////////////////////
    // GUI parameters
    std::string initial_pose_;
    //////////////////////////////
};

#endif
