#include "motion_tracer_ros2/lower_controller.hpp"

LowerController::LowerController() :
    Node("lower_controller_node"),lifter_ratio_(0.002), lifter_forward_lean(false), is_halfway_angle(false) {
    controller_rate_ = 100;
    controller_cycle_ = (1.0/controller_rate_);

    auto teleop_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto notify_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    lifter_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("lifter_controller/joint_trajectory", 2);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/mechanum_controller/cmd_vel_teleop_raw", 2);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("tracer_joy", teleop_qos, std::bind(&LowerController::getJoy, this, std::placeholders::_1));
    forward_lean_sub_ = this->create_subscription<std_msgs::msg::Bool>("/on_lifter_forward_lean", notify_qos, std::bind(&LowerController::notifyForwardLean, this, std::placeholders::_1));

    init_follow_joint_trajectory();
    last_time_ = now();
}

LowerController::~LowerController() {
}

void LowerController::init_follow_joint_trajectory() {
    lifter_msg.joint_names.resize(2);
    lifter_msg.joint_names[0] = "knee_joint";
    lifter_msg.joint_names[1] = "ankle_joint";
    lifter_msg.points.resize(1);
    lifter_msg.points[0].positions.resize(lifter_msg.joint_names.size());
}

void LowerController::sendJointAngles() {
    lifter_msg.points[0].positions = {
        joint_angles_["knee_joint"],
        joint_angles_["ankle_joint"]
    };
    lifter_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(controller_cycle_);

    lifter_traj_pub_->publish(lifter_msg);
}

double LowerController::limitRate(double target, double current, double max_acc, double dt) {
    double diff = target - current;
    double max_step = max_acc * dt;

    if (diff > max_step) {
        diff = max_step;
    } else if (diff < -max_step) {
        diff = -max_step;
    }
    return current + diff;
}

void LowerController::getJoy(const sensor_msgs::msg::Joy::SharedPtr _data) {
    if ((_data->axes[4] == 1 || _data->buttons[6] == 1) && _data->axes[2] != 0) {
        if (!is_halfway_angle || !lifter_forward_lean) {
            joint_angles_["knee_joint"] += (_data->axes[2] * lifter_ratio_);
            if (joint_angles_["knee_joint"] > knee_upper_limt) {
                joint_angles_["knee_joint"] = knee_upper_limt;
            } else if (joint_angles_["knee_joint"] < knee_lower_limt) {
                joint_angles_["knee_joint"] = knee_lower_limt;
                is_halfway_angle = true;
            }
        }
        if (is_halfway_angle || !lifter_forward_lean) {
            joint_angles_["ankle_joint"] -= (_data->axes[2] * lifter_ratio_);
            if (joint_angles_["ankle_joint"] > ankle_upper_limt) {
                joint_angles_["ankle_joint"] = ankle_upper_limt;
            } else if (joint_angles_["ankle_joint"] < ankle_lower_limt) {
                joint_angles_["ankle_joint"] = ankle_lower_limt;
                is_halfway_angle = false;
            }
        }
        sendJointAngles();
    }

    // if ((_data->axes[4] == 1 || _data->buttons[6] == 1) && (std::abs(_data->axes[0]) > 0.05 || std::abs(_data->axes[1]) > 0.05 || std::abs(_data->axes[3]) > 0.05)) {
    //     cmd_vel_.linear.x = 0;
    //     cmd_vel_.linear.y = 0;
    //     cmd_vel_.linear.z = 0;
    //     cmd_vel_.angular.x = 0;
    //     cmd_vel_.angular.y = 0;
    //     cmd_vel_.angular.z = 0;

    //     if (_data->axes[1] != 0) {
    //         cmd_vel_.linear.x = 0.2 * _data->axes[1];
    //     }
    //     if (_data->axes[3] != 0) {
    //         cmd_vel_.linear.y = 0.2 * _data->axes[3];
    //     }
    //     if (_data->axes[0] != 0) {
    //         cmd_vel_.angular.z = 0.5 * _data->axes[0];
    //     }
    //     cmd_vel_pub_->publish(cmd_vel_);
    // }

    // 加速度リミット暫定処理
    if ((_data->axes[4] == 1 || _data->buttons[6] == 1)){
        auto now_time = now();
        double dt = (now_time - last_time_).seconds();
        last_time_ = now_time;

        double target_vx = 0.0;
        double target_vy = 0.0;
        double target_wz = 0.0;

        if (std::abs(_data->axes[1]) > 0.05) {
            target_vx = 0.4 * _data->axes[1];
        }
        if (std::abs(_data->axes[3]) > 0.05) {
            target_vy = 0.2 * _data->axes[3];
        }
        if (std::abs(_data->axes[0]) > 0.05) {
            target_wz = 0.5 * _data->axes[0];
        }

        current_vx_ = limitRate(target_vx, current_vx_, max_linear_acc, dt);
        current_vy_ = limitRate(target_vy, current_vy_, max_linear_acc, dt);
        current_wz_ = limitRate(target_wz, current_wz_, max_angular_acc, dt);

        cmd_vel_.linear.x = current_vx_;
        cmd_vel_.linear.y = current_vy_;
        cmd_vel_.angular.z = current_wz_;

        cmd_vel_pub_->publish(cmd_vel_);
    }
}

void LowerController::notifyForwardLean(const std_msgs::msg::Bool& msg) {
    lifter_forward_lean = msg.data;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LowerController>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
