#include "motion_tracer_ros2/upper_controller.hpp"

UpperController::UpperController() : 
    Node("upper_controller_node"),r_hand_state("open"),l_hand_state("open") {
    controller_rate_ = 50;
    controller_cycle_ = (1.0/controller_rate_);
    move_time_ = controller_cycle_;

    this->declare_parameter<std::string>("neck_movement", "increment");
    this->declare_parameter<int>("neck_offset", 0);
    this->declare_parameter<bool>("neck_reverse", false);
    this->declare_parameter<bool>("neck_auto", false);
    neck_movement_ = this->get_parameter("neck_movement").as_string();
    neck_offset_ = this->get_parameter("neck_offset").as_int();
    neck_reverse_ = this->get_parameter("neck_reverse").as_bool();
    neck_auto_ = this->get_parameter("neck_auto").as_bool();

    tracer_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/tracer_states", 1, std::bind(&UpperController::tracerStateCallback, this, std::placeholders::_1));
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("tracer_joy", 1, std::bind(&UpperController::getJoy, this, std::placeholders::_1));

    grasp_client_ = this->create_client<aero_controller_msgs::srv::RunScript>("/aero_controller/run_script");

    rarm_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("rarm_controller/joint_trajectory", 1);
    larm_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("larm_controller/joint_trajectory", 1);
    waist_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("waist_controller/joint_trajectory", 1);
    head_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("head_controller/joint_trajectory", 1);
    rhand_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("rhand_controller/joint_trajectory", 1);
    lhand_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("lhand_controller/joint_trajectory", 1);

    init_follow_joint_trajectory();
}

UpperController::~UpperController() {
}

void UpperController::init_follow_joint_trajectory() {
    while (!grasp_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for hand grasp service...");
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Grasp Service not available.");
            return;
        }
    }

    rarm_msg.joint_names.resize(7);
    rarm_msg.joint_names[0] = "r_shoulder_p_joint";
    rarm_msg.joint_names[1] = "r_shoulder_r_joint";
    rarm_msg.joint_names[2] = "r_shoulder_y_joint";
    rarm_msg.joint_names[3] = "r_elbow_joint";
    rarm_msg.joint_names[4] = "r_wrist_y_joint";
    rarm_msg.joint_names[5] = "r_wrist_r_joint";
    rarm_msg.joint_names[6] = "r_wrist_p_joint";
    rarm_msg.points.resize(1);
    rarm_msg.points[0].positions.resize(rarm_msg.joint_names.size());

    larm_msg.joint_names.resize(7);
    larm_msg.joint_names[0] = "l_shoulder_p_joint";
    larm_msg.joint_names[1] = "l_shoulder_r_joint";
    larm_msg.joint_names[2] = "l_shoulder_y_joint";
    larm_msg.joint_names[3] = "l_elbow_joint";
    larm_msg.joint_names[4] = "l_wrist_y_joint";
    larm_msg.joint_names[5] = "l_wrist_r_joint";
    larm_msg.joint_names[6] = "l_wrist_p_joint";
    larm_msg.points.resize(1);
    larm_msg.points[0].positions.resize(larm_msg.joint_names.size());

    waist_msg.joint_names.resize(3);
    waist_msg.joint_names[0] = "waist_y_joint";
    waist_msg.joint_names[1] = "waist_p_joint";
    waist_msg.joint_names[2] = "waist_r_joint";
    waist_msg.points.resize(1);
    waist_msg.points[0].positions.resize(waist_msg.joint_names.size());

    head_msg.joint_names.resize(3);
    head_msg.joint_names[0] = "neck_y_joint";
    head_msg.joint_names[1] = "neck_p_joint";
    head_msg.joint_names[2] = "neck_r_joint";
    head_msg.points.resize(1);
    head_msg.points[0].positions.resize(head_msg.joint_names.size());

    rhand_msg.joint_names.resize(1);
    rhand_msg.joint_names[0] = "r_thumb_joint";
    rhand_msg.points.resize(1);
    rhand_msg.points[0].positions.resize(rhand_msg.joint_names.size());

    lhand_msg.joint_names.resize(1);
    lhand_msg.joint_names[0] = "l_thumb_joint";
    lhand_msg.points.resize(1);
    lhand_msg.points[0].positions.resize(lhand_msg.joint_names.size());
}

void UpperController::graspControl(std::string _position, std::string _pose) {
    std::lock_guard<std::mutex> guard(msg_mtx);

    aero_controller_msgs::msg::ScriptReqJNoInterf param;
    if (_position == "right") {
        param.send_no = 11;
    } else if (_position == "left") {
        param.send_no = 26;
    }
    if (_pose == "grasp") {
        param.script_no = 2;
        param.dio_run = 2;
    } else if (_pose == "release") {
        param.script_no = 3;
        param.dio_run = 3;
    }
    param.msid = 1;
    param.arg = 0;

    auto request = std::make_shared<aero_controller_msgs::srv::RunScript::Request>();
    request->jno_interf.clear();
    request->timeout_sec = 3;
    request->jno_interf.push_back(param);

    grasp_client_->async_send_request(request);

    if (_position == "right" && _pose == "grasp") {
        r_hand_state = "close";
    } else if (_position == "right" && _pose == "release") {
        r_hand_state = "open";
    }

    if (_position == "left" && _pose == "grasp") {
        l_hand_state = "close";
    } else if (_position == "left" && _pose == "release") {
        l_hand_state = "open";
    }
}

void UpperController::tracerStateCallback(const sensor_msgs::msg::JointState& _tracer_data) {
    joint_angles_["r_shoulder_p_joint"] = _tracer_data.position[1] / 10.0 * deg2Rad * -1.0;
    joint_angles_["r_shoulder_r_joint"] = (_tracer_data.position[2] / 10.0 - 10) * deg2Rad * -1.0;
    joint_angles_["r_shoulder_y_joint"] = _tracer_data.position[3] / 10.0 * deg2Rad * -1.0;
    joint_angles_["r_elbow_joint"] = _tracer_data.position[4] / 10.0 * deg2Rad;
    joint_angles_["r_wrist_y_joint"] = _tracer_data.position[5] / 10.0 * deg2Rad * -1.0;
    joint_angles_["r_wrist_r_joint"] = _tracer_data.position[6] / 10.0 * deg2Rad;
    joint_angles_["r_wrist_p_joint"] = _tracer_data.position[7] / 10.0 * deg2Rad * -1.0;
    joint_angles_["r_thumb_joint"] = calcHandAngle(_tracer_data.position[8] / 10.0, 0.0);

    joint_angles_["l_shoulder_p_joint"] = _tracer_data.position[9] / 10.0 * deg2Rad;
    joint_angles_["l_shoulder_r_joint"] = (_tracer_data.position[10] / 10.0  + 10) * deg2Rad * -1.0;
    joint_angles_["l_shoulder_y_joint"] = _tracer_data.position[11] / 10.0 * deg2Rad * -1.0;
    joint_angles_["l_elbow_joint"] = _tracer_data.position[12] / 10.0 * deg2Rad * -1.0;
    joint_angles_["l_wrist_y_joint"] = _tracer_data.position[13] / 10.0 * deg2Rad * -1.0;
    joint_angles_["l_wrist_r_joint"] = _tracer_data.position[14] / 10.0 * deg2Rad * -1.0;
    joint_angles_["l_wrist_p_joint"] = _tracer_data.position[15] / 10.0 * deg2Rad;
    joint_angles_["l_thumb_joint"] = calcHandAngle(_tracer_data.position[16] / 10.0, 0.0);

    sendJointAngles();
}

double UpperController::calcHandAngle(double _position, double offset){
    // // (試作1暫定対応)
    // // (リーダー現在角度 - リーダー閉じきった際の角度) / リーダー全移動角度量 * フォロワー全移動角度量 + フォロワー閉じきった際の角度 + offset;
    // return (_position + 11) / 56  * 0.15 - 0.06462 + offset;
    // (試作2暫定対応)
    return (_position + 45) / 90 * 0.15 - 0.06462 + offset;
}

void UpperController::sendJointAngles() {
    rarm_msg.points[0].positions = {
        joint_angles_["r_shoulder_p_joint"],
        joint_angles_["r_shoulder_r_joint"],
        joint_angles_["r_shoulder_y_joint"],
        joint_angles_["r_elbow_joint"],
        joint_angles_["r_wrist_y_joint"],
        joint_angles_["r_wrist_r_joint"],
        joint_angles_["r_wrist_p_joint"]
    };
    rarm_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(controller_cycle_);

    larm_msg.points[0].positions = {
        joint_angles_["l_shoulder_p_joint"],
        joint_angles_["l_shoulder_r_joint"],
        joint_angles_["l_shoulder_y_joint"],
        joint_angles_["l_elbow_joint"],
        joint_angles_["l_wrist_y_joint"],
        joint_angles_["l_wrist_r_joint"],
        joint_angles_["l_wrist_p_joint"]
    };
    larm_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(controller_cycle_);

    waist_msg.points[0].positions = {
        joint_angles_["waist_y_joint"],
        joint_angles_["waist_p_joint"],
        joint_angles_["waist_r_joint"]   
    };
    waist_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(controller_cycle_);

    head_msg.points[0].positions = {
        joint_angles_["neck_y_joint"],
        joint_angles_["neck_p_joint"],
        joint_angles_["neck_r_joint"]
    };
    head_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(controller_cycle_);

    rhand_msg.points[0].positions = {
        joint_angles_["r_thumb_joint"]
    };
    rhand_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(controller_cycle_);

    lhand_msg.points[0].positions = {
        joint_angles_["l_thumb_joint"]
    };
    lhand_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(controller_cycle_);

    rarm_traj_pub_->publish(rarm_msg);
    larm_traj_pub_->publish(larm_msg);
    waist_traj_pub_->publish(waist_msg);
    head_traj_pub_->publish(head_msg);
    rhand_traj_pub_->publish(rhand_msg);
    lhand_traj_pub_->publish(lhand_msg);

}

void UpperController::getJoy(const sensor_msgs::msg::Joy::SharedPtr _data) {
    double look_right = 0;
    double look_left = 0;

    //If you don't want to look at hand, you should comment out below.
    look_right = -joint_angles_["r_shoulder_r_joint"] - joint_angles_["r_shoulder_y_joint"];
    look_left = joint_angles_["l_shoulder_r_joint"] + joint_angles_["l_shoulder_y_joint"];

    int sign = 1;
    if (neck_reverse_) {
        sign = -1;
    } else {
        sign = 1;
    }

    // for waist pitch & roll
    if (_data->axes[4] == 0 && (std::abs(_data->axes[0]) > 0.05 || std::abs(_data->axes[1]) > 0.05)) {
        if (neck_movement_ == "absolute") {
            joint_angles_["waist_y_joint"] = (_data->axes[0] * 1) ;
            joint_angles_["waist_p_joint"] = sign * (_data->axes[1] * -1) ;
        } else if (neck_movement_ == "increment") {
            joint_angles_["waist_y_joint"] += sign * (_data->axes[0] * 0.05);
            joint_angles_["waist_p_joint"] -= sign * (_data->axes[1] * 0.05);
        }
    } else {
        if (neck_movement_ == "absolute") {
            joint_angles_["waist_y_joint"] = 0;
            joint_angles_["waist_p_joint"] = 0;
        }
    }

    //for neck pitch & roll
    if (_data->axes[4] == 0 && (std::abs(_data->axes[3]) > 0.05 || std::abs(_data->axes[2]) > 0.05)) {
        if(neck_movement_ == "absolute") {
            joint_angles_["neck_y_joint"] = (_data->axes[3] * 2);
            joint_angles_["neck_p_joint"] = neck_offset_*(M_PI/180) + sign * (_data->axes[2] * -2);
        } else if (neck_movement_ == "increment") {
            joint_angles_["neck_y_joint"] += (_data->axes[3] * 0.05);
            joint_angles_["neck_p_joint"] -= sign * (_data->axes[2] * 0.05);
        }
    } else { //in case of automatically moving neck  or not
        //std::cout << neck_auto_  << "," << look_right << "," << look_left << std::endl;
        if ((neck_auto_ == true) && (look_right > look_left) && (look_right > 0.5)) {
            //std::cout << "Right Look" << std::endl;
            joint_angles_["neck_y_joint"] = joint_angles_["r_shoulder_y_joint"];   //assign r_shoulder_y
            if (joint_angles_["r_shoulder_r_joint"] > -0.79) {
                joint_angles_["neck_p_joint"] = (2.36 + joint_angles_["r_elbow_joint"])/2; //assign r_elbow when r_shoulder_r is low
            } else {
                joint_angles_["neck_p_joint"] = (1.05 + joint_angles_["r_shoulder_r_joint"]); //assign r_shoulder_r when r_shoulder_r is high
            }
        } else if ((neck_auto_ == true) && (look_right < look_left) && (look_left > 0.5)) {
            //std::cout << "Left Look" << std::endl;
            joint_angles_["neck_y_joint"] = joint_angles_["l_shoulder_y_joint"];  //assign l_shoulder_y
            if (joint_angles_["l_shoulder_r_joint"] < 0.79) {
                joint_angles_["neck_p_joint"] = (2.36 + joint_angles_["l_elbow_joint"])/2; //assign l_elbow when l_shoulder_r is low
            } else {
                joint_angles_["neck_p_joint"] = (1.05 - joint_angles_["l_shoulder_r_joint"]); //assign l_shoulder_r when l_shoulder_r is high
            }
        } else if (neck_movement_ == "absolute") {
            joint_angles_["neck_y_joint"] = 0;
            joint_angles_["neck_p_joint"] = neck_offset_*(M_PI/180);
        }
    }

    // set angle limit in joy stick data
    if (joint_angles_["waist_y_joint"] > waist_y_upper_limt) {
        joint_angles_["waist_y_joint"] = waist_y_upper_limt;
    } else if (joint_angles_["waist_y_joint"] < waist_y_lower_limt) {
        joint_angles_["waist_y_joint"] = waist_y_lower_limt;
    }

    if (joint_angles_["waist_p_joint"] > waist_p_upper_limt) {
        joint_angles_["waist_p_joint"] = waist_p_upper_limt;
    } else if (joint_angles_["waist_p_joint"] < waist_p_lower_limt) {
        joint_angles_["waist_p_joint"] = waist_p_lower_limt;
    }

    if (joint_angles_["neck_r_joint"] > neck_r_upper_limt) {
        joint_angles_["neck_r_joint"] = neck_r_upper_limt;
    } else if (joint_angles_["neck_r_joint"] < neck_r_lower_limt) {
        joint_angles_["neck_r_joint"] = neck_r_lower_limt;
    }

    if (joint_angles_["neck_y_joint"] > neck_y_upper_limt) {
        joint_angles_["neck_y_joint"] = neck_y_upper_limt;
    } else if (joint_angles_["neck_y_joint"] < neck_y_lower_limt) {
        joint_angles_["neck_y_joint"] = neck_y_lower_limt;
    }

    if (joint_angles_["neck_p_joint"] > neck_p_upper_limt) {
        joint_angles_["neck_p_joint"] = neck_p_upper_limt;
    } else if (joint_angles_["neck_p_joint"] < neck_p_lower_limt) {
        joint_angles_["neck_p_joint"] = neck_p_lower_limt;
    }

    // set initial pose origin or MC
    if (_data->axes[4] == -1 || _data->axes[4] == 2 || _data->buttons[0] == 1) {
        joint_angles_["waist_y_joint"] = 0;
        joint_angles_["waist_p_joint"] = 0;
        joint_angles_["waist_r_joint"] = 0;
        joint_angles_["neck_y_joint"] = 0;
        joint_angles_["neck_r_joint"] = 0;
        joint_angles_["neck_p_joint"] = neck_offset_*(M_PI/180);

        sendJointAngles();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UpperController>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}