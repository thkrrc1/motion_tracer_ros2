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
        double fx = 0.0;
        double fy = 0.0;
        double fz = 0.0;
        double mx = 0.0;
        double my = 0.0;
        double mz = 0.0;
    };

    struct KdlActiveJointMapping
    {
        std::string kdl_joint_name;
        std::string active_joint_name;
        int active_index = -1;
        double multiplier = 1.0;
        double offset = 0.0;
    };

    struct ArmContext
    {
        std::string side;
        std::string joint_prefix;

        // Sensor parameters/state
        std::string device;
        std::string acquisition_mode = "continuous";
        std::string sensor_frame_id;
        int fd = -1;
        std::mutex serial_mutex;
        std::vector<uint8_t> continuous_rx_buffer;
        std::atomic<bool> continuous_output_started = false;
        std::atomic<bool> sensor_ready = false;
        std::atomic<bool> tare_requested = false;
        std::array<double, 6> calib = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        Wrench6 filtered;
        std::atomic<bool> filter_initialized = false;

        double payload_mass_kg = 0.82 + 0.25; // hand unit[kg] + force sensor[kg]
        std::vector<double> payload_com_in_sensor = {0.0, 0.0, 0.063}; // センサー座標系からみたハンドユニット重心ベクトル
        std::vector<double> gravity_vector_in_base = {0.0, 0.0, -9.80665}; // ワールド座標系からみた重力ベクトル
        double gravity_compensation_sign = 1.0;
        Wrench6 gravity_reference_wrench;
        std::atomic<bool> gravity_reference_valid = false;
        std::atomic<bool> gravity_reference_pending = true;

        // Kinematics parameters/state
        std::string base_link;
        std::string tip_link;
        std::vector<double> sensor_xyz_in_tip = {0.0, 0.0, 0.0}; // tip_link座標系からみたセンサー検出位置
        std::vector<double> sensor_rpy_in_tip = {0.0, 0.0, 0.0}; // tip_link座標系からみたセンサー検出方向
        std::vector<std::string> active_joint_names;
        KDL::Frame tip_T_sensor;

        std::mutex kdl_mutex;
        KDL::Chain chain;
        std::vector<std::string> raw_kdl_joint_names;
        std::vector<KdlActiveJointMapping> kdl_active_mapping;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
        std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;
        std::atomic<bool> kinematics_ready = false;

        // Collision / filtering
        double force_threshold = 5.0;
        double torque_threshold = 1.0;
        double force_deadband = 1.0;
        double torque_deadband = 0.02;
        double tau_deadband = 0.2;
        double lowpass_alpha = 0.35;

        // Current reflection
        std::vector<int64_t> arm_joint_indices;
        std::vector<double> current_signs;
        std::vector<double> current_gains;
        int max_current_delta = 600;
        double max_delta_rate_per_sec = 6000.0;
        std::vector<double> last_current_delta;
        rclcpp::Time last_current_publish_time;

        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr compensated_wrench_pub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tau_pub;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_service;
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

    void configureArm(
        ArmContext & arm, const std::string & side, const std::string & joint_prefix,
        const std::string & default_device, const std::string & default_base_link, const std::string & default_tip_link,
        const std::vector<double> & default_sensor_xyz_in_tip, const std::vector<double> & default_sensor_rpy_in_tip,
        const std::vector<std::string> & default_active_joint_names,
        const std::vector<double> & default_current_signs, const std::vector<double> & default_current_gains,
        const std::vector<int64_t> & default_arm_indices,
        const std::vector<double> & default_payload_com_in_sensor, const std::vector<double> & default_gravity_vector_in_base,
        const double & default_gravity_compensation_sign
    );
    bool validateArmConfiguration(const ArmContext & arm) const;

    // ---- Serial ----
    bool initializeSensor(ArmContext & arm);
    bool openSerial(ArmContext & arm);
    void closeSerial(ArmContext & arm);
    void flushSerial(ArmContext & arm);
    bool writeAll(ArmContext & arm, const char * data, size_t len);
    bool readCalibText(ArmContext & arm, std::string& line, int timeout_ms, size_t max_len);
    bool readCalibration(ArmContext & arm);
    bool startStream(ArmContext & arm);
    bool startStreamUnlocked(ArmContext & arm);
    bool stopStream(ArmContext & arm);
    bool stopStreamUnlocked(ArmContext & arm);
    bool handleTare(ArmContext & arm);
    bool readExact(ArmContext & arm, uint8_t *dst, size_t len, int timeout_ms);
    bool readSampleRequestResponse(ArmContext & arm, Wrench6 & out);
    bool extractLatestFrame(ArmContext & arm, std::array<uint8_t, WRENCH_DATA_LENGTH> & latest_frame);
    bool readStream(ArmContext & arm, int timeout_ms);
    bool readSampleContinuous(ArmContext & arm, Wrench6 & out);
    bool readSample(ArmContext & arm, Wrench6 & out);
    bool parseFrame(ArmContext & arm, const uint8_t* frame, Wrench6& out);
    bool parseWrench(ArmContext & arm, const char *payload, Wrench6& out);

    bool buildKinematicsFromUrdfForArm(ArmContext & arm, const std::string & robot_description);
    bool buildDefaultNoidArmJointMapping(const ArmContext & arm, const std::vector<std::string> & raw_kdl_joint_names, std::vector<KdlActiveJointMapping> & mapping);
    bool getActiveJointPositions(const ArmContext & arm, std::vector<double> & active_q);
    int findActiveJointIndex(const ArmContext & arm, const std::string & name);
    KDL::Frame makeSensorFrameFromParams(const ArmContext & arm) const;
    static KDL::Vector cross(const KDL::Vector & a, const KDL::Vector & b);
    bool computeTauFromWrench(ArmContext & arm, const Wrench6 & sensor_wrench, std::vector<double> & tau_out);
    bool computeGravityWrenchInSensor(ArmContext & arm, Wrench6 & gravity_wrench);
    bool compensatePayloadGravity(ArmContext & arm, const Wrench6 & measured, Wrench6 & compensated);
    static Wrench6 subtractWrench(const Wrench6 & lhs, const Wrench6 & rhs);
    static Wrench6 scaleWrench(const Wrench6 & wrench, double scale);

    // ---- Current mux ----
    aero_controller_msgs::msg::Current makeOutputFromBaseOrNeutral();
    bool isProtectedTracerIndex(int tracer_index) const;
    void wrenchLoop();
    bool processArmSample(ArmContext & arm, std::vector<double> & tau_out);
    void resetReflectionDelta(ArmContext & arm);
    void resetAllReflectionDeltas();
    void publishPassThroughOrNeutral();
    void publishNeutralCurrent();
    void publishMergedCurrent(const std::vector<double> * right_tau, bool right_has_reflection, const std::vector<double> * left_tau, bool left_has_reflection);
    void publishWrenchDebug(ArmContext & arm, const Wrench6 & wrench, const std::string & frame_id);
    void publishTauDebug(ArmContext & arm, const std::vector<double> & tau, const std::vector<std::string> & names);
    void mergeArmCurrent(aero_controller_msgs::msg::Current & msg, ArmContext & arm, const std::vector<double> & tau);
    void fillNeutral(aero_controller_msgs::msg::Current & msg) const;
    int convertCurrentValue(int current_val);
    void setCurrentWord(aero_controller_msgs::msg::Current & msg, int current_index, uint16_t raw) const;
    uint16_t clampCurrent(int value) const;
    static double applyDeadband(double value, double deadband);
    static double norm3(double x, double y, double z);
    std::string joinStrings(const std::vector<std::string> & values);

    // ---- ROS interfaces ----
    rclcpp::Subscription<aero_controller_msgs::msg::Current>::SharedPtr current_raw_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Publisher<aero_controller_msgs::msg::Current>::SharedPtr current_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr on_force_feedback_sub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sensor_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_srv_;

    ArmContext right_arm_;
    ArmContext left_arm_;

    /// Common parameters
    size_t retry_calib_max_count = 3;
    double lbs_val = 8192.0;
    int frame_rx_buffer_max_bytes = 4096;
    int frame_rx_buffer_keep_bytes = 512;
    double sample_rate_hz = 100.0;
    bool publish_wrench_debug = true;
    bool publish_tau_debug = true;

    uint16_t neutral_current = 0x0000;
    int right_hand_aero_id = 7;
    int left_hand_aero_id = 22;
    std::vector<int64_t> protected_arm_joint_indices = {7, 22, 28, 29};

    std::thread wrench_thread_;
    std::atomic<bool> running_ = false;
    std::atomic<bool> on_force_feedback_ = true;

    // ---- current raw state ----
    std::mutex current_raw_mutex;
    aero_controller_msgs::msg::Current latest_current_raw;
    rclcpp::Time latest_current_raw_stamp;
    std::atomic<bool> have_current_raw = false;

    std::mutex joint_mutex;
    std::unordered_map<std::string, double> latest_joint_position;
    rclcpp::Time latest_joint_stamp;
    std::atomic<bool> have_joint_state = false;
};

#endif  // WRENCH_CONTROLLER_HPP_