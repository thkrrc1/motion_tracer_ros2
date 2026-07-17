#ifndef TRACER_TELEOP_HPP_
#define TRACER_TELEOP_HPP_

#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "motion_tracer_ros2/tracer_command.hpp"

#include "aero_controller_msgs/msg/current.hpp"

#include "std_msgs/msg/bool.hpp"

class TracerTeleop : public rclcpp::Node
{
public:
    TracerTeleop();
    ~TracerTeleop();

    void currentCallback(const aero_controller_msgs::msg::Current& _current_data);
    void processLoop();
    void processPacket(std::vector<uint8_t>& packet);

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

    rclcpp::Subscription<aero_controller_msgs::msg::Current>::SharedPtr current_sub_;

    std::vector<uint8_t> latest_current_;
    std::vector<uint16_t> latest_pos;
    std::mutex current_mutex_;

    std::thread tx_thread_;
    std::atomic<bool> running_;
    void txLoop();

    bool wheel_stop_flag_;

    int init_counter_;
    bool tracer_mode_;

    bool only_hand_current = true;
    int right_hand_aero_id = 7;
    int left_hand_aero_id = 22;
    double scale = 1.0;

    tracer::controller::FootPedalCommand *foot_pedal_;
    void updateFootPedalInput();
    bool getFootPedalInput(std::string& input);
    bool foot_pedal_enabled = true;
    std::mutex foot_pedal_input_mutex_;
    std::string latest_foot_pedal_input_;
    bool is_mover_mode;
    bool on_tracer_mode;

    void notifyTracerMode();
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tracer_mode_pub_;
    void notifyOnTracer();
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr on_tracer_pub_;

    //////////////////////////////
    // GUI parameters
    std::string initial_pose_;
    //////////////////////////////
};

#endif
