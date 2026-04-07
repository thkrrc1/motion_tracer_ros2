#include "motion_tracer_ros2/lower_controller.hpp"

LowerController::LowerController() :
    Node("lower_controller_node"),lifter_ratio_(0.01),update_joints(true) {
    controller_rate_ = 50;
    controller_cycle_ = (1.0/controller_rate_);
    move_time_ = controller_cycle_;

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&LowerController::getJoy, this, std::placeholders::_1));

    lifter_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("lifter_controller/joint_trajectory", 1);

    init_follow_joint_trajectory();
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

void LowerController::getJoy(const sensor_msgs::msg::Joy::SharedPtr _data) {
    if ((_data->buttons[4] == 1 || _data->buttons[6] == 1) && _data->axes[2] != 0){
        update_joints = false;
        joint_angles_["ankle_joint"] -= (_data->axes[2] * lifter_ratio_);
        joint_angles_["knee_joint"] += (_data->axes[2] * lifter_ratio_);

        if (joint_angles_["ankle_joint"] > ankle_upper_limt) {
            joint_angles_["ankle_joint"] = ankle_upper_limt;
        } else if (joint_angles_["ankle_joint"] < ankle_lower_limt) {
            joint_angles_["ankle_joint"] = ankle_lower_limt;
        }
        if (joint_angles_["knee_joint"] > knee_upper_limt) {
            joint_angles_["knee_joint"] = knee_upper_limt;
        } else if (joint_angles_["knee_joint"] < knee_lower_limt) {
            joint_angles_["knee_joint"] = knee_lower_limt;
        }

        sendJointAngles();
    } else {
        update_joints = true;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LowerController>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
