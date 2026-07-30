#ifndef WRENCH_CONTROLLER_HPP_
#define WRENCH_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include "aero_controller_msgs/msg/current.hpp"

class WrenchController : public rclcpp::Node
{
public:
    WrenchController();
    ~WrenchController();

private:
    struct Wrench6 {
        double fx{0.0};
        double fy{0.0};
        double fz{0.0};
        double mx{0.0};
        double my{0.0};
        double mz{0.0};
    };

    static constexpr int MOTOR_COUNT = 30;
    static constexpr size_t WRENCH_DATA_LENGTH = 27;
    static constexpr size_t WRENCH_PAYLOAD_DATA_LENGTH = 25;
    static constexpr uint8_t CR = 0x0D;
    static constexpr uint8_t LF = 0x0A;

    // ---- ROS callbacks ----
    void currentRawCallback(const aero_controller_msgs::msg::Current::SharedPtr msg);
    void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void onForceFeedbackCallback(const std_msgs::msg::Bool & msg);
    void tareCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // ---- Serial ----
    bool openSerial(std::string device_);
    void closeSerial();
    void flushSerial();
    bool writeAll(const char * data, size_t len);
    bool readCalibText(std::string& line, int timeout_ms, size_t max_len);
    bool readCalibration();
    bool startStream();
    bool stopStream();
    bool handleTare();
    // bool setFrequencyDivider(int div);
    bool readExact(uint8_t *dst, size_t len, int timeout_ms);
    bool readSampleRequestResponse(Wrench6 & out);
    bool extractLatestFrame(std::array<uint8_t, WRENCH_DATA_LENGTH> & latest_frame);
    bool readStream(int timeout_ms);
    bool readSampleContinuous(Wrench6 & out);
    bool readSample(Wrench6 & out);
    bool parseFrame(const uint8_t* frame, Wrench6& out);
    bool parseWrench(const char *payload, Wrench6& out);

    // ---- Kinematics ----
    struct KdlActiveJointMapping
    {
        std::string kdl_joint_name;
        std::string active_joint_name;
        int active_index{-1};
        double multiplier{1.0};
        double offset{0.0};
    };
    
    bool buildKinematicsFromUrdf(const std::string & robot_description);
    bool buildDefaultNoidRightArmJointMapping(const std::vector<std::string> & raw_kdl_joint_names, std::vector<KdlActiveJointMapping> & mapping);
    bool buildConfiguredJointMapping(const std::vector<std::string> & raw_kdl_joint_names, std::vector<KdlActiveJointMapping> & mapping);
    bool getActiveJointPositions(std::vector<double> & active_q, std::vector<std::string> & names);
    bool getCurrentJointArray(KDL::JntArray & q, std::vector<std::string> & names);
    int findActiveJointIndex(const std::string & name);
    KDL::Frame makeSensorFrameFromParams() const;
    static KDL::Vector cross(const KDL::Vector & a, const KDL::Vector & b);
    bool computeTauFromWrench(const Wrench6 & sensor_wrench, std::vector<double> & tau_out);

    // ---- Current mux ----
    aero_controller_msgs::msg::Current makeOutputFromBaseOrNeutral();
    bool isProtectedTracerIndex(int tracer_index) const;
    void wrenchLoop();
    void resetReflectionDelta();
    void publishPassThroughOrNeutral();
    void publishNeutralCurrent();
    void publishMergedCurrent(const std::vector<double> * tau, bool enable_reflection);
    void fillNeutral(aero_controller_msgs::msg::Current & msg) const;
    int convertCurrentValue(int current_val);
    void setCurrentWord(aero_controller_msgs::msg::Current & msg, int current_index, uint16_t raw) const;
    uint16_t clampCurrent(int value) const;
    static double applyDeadband(double value, double deadband);
    static double norm3(double x, double y, double z);
    void publishTauDebug(const std::vector<double> & tau, const std::vector<std::string> & names);
    std::string joinStrings(const std::vector<std::string> & values);

    // ---- ROS interfaces ----
    rclcpp::Subscription<aero_controller_msgs::msg::Current>::SharedPtr current_raw_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Publisher<aero_controller_msgs::msg::Current>::SharedPtr current_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr on_force_feedback_sub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sensor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tau_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_srv_;

    // ---- Parameters: force sensor ----
    size_t retry_calib_max_count = 3;
    double lbs_val = 8192.0;
    std::string acquisition_mode = "continuous";
    int frame_rx_buffer_max_bytes = 4096;
    int frame_rx_buffer_keep_bytes = 512;
    std::array<double, 6> calib_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    // int frequency_divider_ = 1;

    // ---- Parameters: frames / KDL ----
    std::string base_link_="r_arm_link";
    std::string tip_link_= "r_hand_link";
    std::vector<double> sensor_xyz_in_tip = {0.0, 0.0, 0.0};
    std::vector<double> sensor_rpy_in_tip = {0.0, 0.0, 0.0};
    std::vector<std::string> active_joint_names = {
        "r_shoulder_p_joint",
        "r_shoulder_r_joint",
        "r_shoulder_y_joint",
        "r_elbow_joint",
        "r_wrist_y_joint",
        "r_wrist_r_joint",
        "r_wrist_p_joint"
    };
    // std::vector<std::string> configured_kdl_joint_names_;
    // std::vector<std::string> configured_active_joint_names_;
    // std::vector<double> configured_kdl_to_active_multipliers_;
    // std::vector<double> configured_kdl_to_active_offsets_;

    // ---- Parameters: collision ----
    double sample_rate_hz = 100.0;
    bool publish_wrench_debug = true;
    bool publish_tau_debug = true;
    double force_threshold = 2.0;
    double torque_threshold = 0.05;
    double force_deadband = 1.0;
    double torque_deadband = 0.03;
    double tau_deadband = 0.02;
    double lowpass_alpha = 0.35;

    // ---- Parameters: current mux / reflection ----
    uint16_t neutral_current = 0x0000;
    int right_hand_aero_id = 7;
    int left_hand_aero_id = 22;
    int max_current_delta = 800;
    double max_delta_rate_per_sec_ = 40000.0;
    std::vector<int64_t> arm_joint_indices = {0, 1, 2, 3, 4, 5, 6};
    std::vector<int64_t> protected_arm_joint_indices = {7, 22, 28, 29};
    std::vector<double> current_signs = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
    std::vector<double> current_gains = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
    std::vector<double> last_current_delta;
    rclcpp::Time last_current_publish_time;

    // ---- Serial state ----
    int fd_;
    std::mutex serial_mutex_;
    std::vector<uint8_t> continuous_rx_buffer_;
    std::atomic<bool> continuous_output_started_ = false;
    std::atomic<bool> tare_requested_ = false;

    // ---- worker state ----
    std::thread wrench_thread_;
    std::atomic<bool> running_ = false;
    std::atomic<bool> on_force_feedback_ = true;
    Wrench6 filtered_;
    bool filter_initialized = false;

    // ---- current raw state ----
    std::mutex current_raw_mutex;
    aero_controller_msgs::msg::Current latest_current_raw;
    rclcpp::Time latest_current_raw_stamp;
    std::atomic<bool> have_current_raw{false};

    // ---- KDL state ----
    std::mutex kdl_mutex;
    KDL::Chain chain_;
    std::vector<std::string> raw_kdl_joint_names_;
    std::vector<KdlActiveJointMapping> kdl_active_mapping_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::atomic<bool> kinematics_ready = false;
    KDL::Frame tip_T_sensor;

    // ---- joint state ----
    std::mutex joint_mutex;
    std::unordered_map<std::string, double> latest_joint_position;
    rclcpp::Time latest_joint_stamp;
    std::atomic<bool> have_joint_state = false;
};

#endif