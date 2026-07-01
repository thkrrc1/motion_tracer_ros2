#include "motion_tracer_ros2/tracer_teleop.hpp"

TracerTeleop::TracerTeleop() :
    Node("tracer_teleop_node"),init_counter_(0),tracer_mode_(0) {

    tracer_ = new tracer::controller::TracerCommand();
    if (!tracer_->port_open("/dev/tracer_usb", 460800)) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed");
        rclcpp::shutdown();
        return;
    }
    wheel_stop_flag_ = true;

    this->declare_parameter<std::string>("initial_pose", "");
    initial_pose_ = this->get_parameter("initial_pose").as_string();

    foot_pedal_ = new tracer::controller::FootPedalCommand();
    if (foot_pedal_enabled) {
        if (!foot_pedal_->device_open("/dev/input/foot_pedal", true)) {
            RCLCPP_WARN(this->get_logger(), "Foot pedal monitoring failed to start");
        }
    }
    is_mover_mode = false;
    on_tracer_mode = false;

    auto teleop_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile().lifespan(std::chrono::milliseconds(100));
    auto notify_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto current_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

    // Publisher
    tracer_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("tracer_states", teleop_qos);
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("tracer_joy", teleop_qos);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", teleop_qos);

    tracer_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/tracer_mode", notify_qos);
    notifyTracerMode();
    on_tracer_pub_ = this->create_publisher<std_msgs::msg::Bool>("/on_tracer", notify_qos);
    notifyOnTracer();

    // Subscriber
    current_sub_ = this->create_subscription<aero_controller_msgs::msg::Current>("/current_controller/current", current_qos, std::bind(&TracerTeleop::currentCallback, this, std::placeholders::_1));

    // JointState初期化
    tracer_state_.name.resize(17);
    tracer_state_.position.resize(17);
    tracer_state_.header.frame_id ="tracer_base";
    // Joy初期化
    joy_.axes.resize(6, 0.0);
    joy_.buttons.resize(13, 0);
    // Tracer position初期化
    position_.resize(30, 0);

    //電流値初期化
    latest_current_.resize(60, 0);
    latest_pos.resize(30, 0);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&TracerTeleop::processLoop, this));
    running_ = true;
    tx_thread_ = std::thread(&TracerTeleop::txLoop, this);
}

TracerTeleop::~TracerTeleop() {
    running_.store(false);

    if(tx_thread_.joinable())
    {
        tx_thread_.join();
    }

    if (foot_pedal_ != nullptr) {
        foot_pedal_->device_close();
        delete foot_pedal_;
        foot_pedal_ = nullptr;
    }

    delete tracer_;
    if (tracer_ != nullptr) {
        tracer_ = nullptr;
    }
}

void TracerTeleop::currentCallback(const aero_controller_msgs::msg::Current& _current_data) {
    std::lock_guard<std::mutex> lock(current_mutex_);
    for (unsigned int idx = 0; idx < _current_data.data.size(); ++idx) {
        latest_current_[idx] = _current_data.data[idx];
    }

    for (unsigned int idx = 0; idx < _current_data.pos_data.size(); ++idx) {
        latest_pos[idx] = _current_data.pos_data[idx];
    }
}

void TracerTeleop::txLoop() {

    rclcpp::WallRate rate(50.0);

    std::vector<uint8_t> current_copy;
    std::vector<uint16_t> pos_copy;
    while(running_) {
        {
            std::lock_guard<std::mutex> lock(current_mutex_);
            current_copy = latest_current_;
            pos_copy = latest_pos;
        }

        if(current_copy.size()!= 60 || pos_copy.size()!=30){
            return;
        }

        std::vector<uint16_t> currents(current_copy.size()/2);

        // 初期化
        for (unsigned int idx = 0; idx < currents.size(); ++idx) {
            currents[idx] = 0x7FFF;
        }

        if (only_hand_current) {
            int right_hand_aero_id = 7;
            int left_hand_aero_id = 22;
            double scale = 1.0;

            uint16_t r_hand_current = static_cast<uint16_t>(current_copy[right_hand_aero_id * 2]) << 8 |
                                        static_cast<uint16_t>(current_copy[right_hand_aero_id * 2 + 1]);
            uint16_t l_hand_current = static_cast<uint16_t>(current_copy[left_hand_aero_id * 2]) << 8 |
                                        static_cast<uint16_t>(current_copy[left_hand_aero_id * 2 + 1]);

            currents[7] = r_hand_current * scale;
            currents[22] = l_hand_current * scale;

            // ハンド位置の送信暫定対応
            currents[28] = pos_copy[right_hand_aero_id];
            currents[29] = pos_copy[left_hand_aero_id];
            
        } else {
            for (unsigned int idx = 0; idx < currents.size(); ++idx) {
                currents[idx] = static_cast<uint16_t>(current_copy[idx * 2]) << 8 |
                                    static_cast<uint16_t>(current_copy[idx * 2 + 1]);
            }
        }
        tracer_->send_current(currents);

        rate.sleep();
    }
}

void TracerTeleop::processLoop() {
    updateFootPedalInput();

    std::vector<uint8_t> packet;

    int max_process = 20; // 安全上限（暴走防止）
    int count = 0;

    // キューを全部処理（最新状態に追従）
    while (tracer_->get_packet(packet) && count < max_process) {
        if (packet.empty()) {
            break;
        }
        processPacket(packet);
        count++;
    }
}

void TracerTeleop::processPacket(std::vector<uint8_t>& tracer_data_) {
    if (tracer_data_.size() < 1) {
        return;
    }

    if (tracer_data_[0] == 0xDF) {
        if (tracer_data_.size() < 64) {
            return;
        }

        for(int i=0;i<30;++i) {
            position_[i] = (tracer_data_[i*2+5] << 8) + tracer_data_[i*2+6];
        }

        // std::cout << "--------Tracer Position--------" << std::endl;
        // for(int i=0;i<30;++i) {
        //     std::cout << "position_["<<i<<"] = " << position_[i]<< std::endl;
        // }
        // std::cout << "-------------------------------" << std::endl;


        //////right arm//////
        // r_shoulder_p_joint
        if (0 <= position_[0] && position_[0] <= 986) {
            tracer_state_.position[1] = position_[0];
        } else if (986 < position_[0] && position_[0] <= 1800) {
            tracer_state_.position[1] = 986;
        }
        // r_shoulder_r_joint
        if (0 <= position_[1] && position_[1] <= 900) {
            tracer_state_.position[2] = position_[1];
        } else if (900 < position_[1] && position_[1] < 1800) {
            tracer_state_.position[2] = 900;
        }
        // r_shoulder_y_joint
        if (0 <= position_[2] && position_[2] <= 1800) {
            tracer_state_.position[3] = position_[2];
        } else if (1800 < position_[2] && position_[2] < 3600) {
            tracer_state_.position[3] = position_[2] - 3600;
        }
        // r_elbow_joint
        if (1880 <= position_[3] && position_[3] < 3600) {
            tracer_state_.position[4] = position_[3] - 3600;
        } else if (900 < position_[3] && position_[3] < 1880) {
            tracer_state_.position[4] = -1720;
        }
        // r_wrist_y_joint
        if (0 <= position_[4] && position_[4] <= 1800) {
            tracer_state_.position[5] = position_[4];
        } else if (1800 < position_[4] && position_[4] < 3600) {
            tracer_state_.position[5] = position_[4] - 3600;
        }
        // r_wrist_r_joint
        if (0 <= position_[5] && position_[5] <= 1800) {
            tracer_state_.position[6] = position_[5];
        } else if (1800 < position_[5] && position_[5] < 3600) {
            tracer_state_.position[6] = position_[5] - 3600;
        }
        // r_wrist_p_joint
        if (0 <= position_[6] && position_[6] <= 1800) {
            tracer_state_.position[7] = position_[6];
        } else if (1800 < position_[6] && position_[6] < 3600) {
            tracer_state_.position[7] = position_[6] - 3600;
        }
        // // r_thumb_joint(試作1暫定対応)
        // if (0 <= position_[7] && position_[7] <= 110) {
        //     tracer_state_.position[8] = position_[7];
        // } else if (3150 <= position_[7] && position_[7] < 3600) {
        //     tracer_state_.position[8] = position_[7] - 3600;
        // } else if (1800 < position_[7] && position_[7] < 3150) {
        //     tracer_state_.position[8] = -450;
        // } else if (110 < position_[7] && position_[7] <= 1800) {
        //     tracer_state_.position[8] = 110;
        // }
        // r_thumb_joint(試作2暫定対応)
        if (2700 <= position_[7] && position_[7] < 3600) {
            tracer_state_.position[8] = 3150 - position_[7];
        } else if (1800 <= position_[7] && position_[7] < 2700) {
            tracer_state_.position[8] = 450;
        } else if (0 <= position_[7] && position_[7] < 1800) {
            tracer_state_.position[8] = -450;
        }

        //////left arm//////
        // l_shoulder_p_joint
        if (2614 <= position_[15] && position_[15] < 3600) {
            tracer_state_.position[9] = position_[15] - 3600;
        } else if (1800 <= position_[15] && position_[15] < 2614) {
            tracer_state_.position[9] = -986;
        }
        // l_shoulder_r_joint
        if (2700 <= position_[16] && position_[16] < 3600) {
            tracer_state_.position[10] = position_[16] - 3600;
        } else if (1800 <= position_[16] && position_[16] < 2700) {
            tracer_state_.position[10] = -900;
        }
        // l_shoulder_y_joint
        if (0 <= position_[17] && position_[17] <= 1800) {
            tracer_state_.position[11] = position_[17];
        } else if (1800 < position_[17] && position_[17] < 3600) {
            tracer_state_.position[11] = position_[17] - 3600;
        }
        // l_elbow_joint
        if (0 <= position_[18] && position_[18] <= 1720) {
            tracer_state_.position[12] = position_[18];
        } else if (1720 < position_[18] && position_[18] < 2700) {
            tracer_state_.position[12] = 1720;
        }
        // l_wrist_y_joint
        if (0 <= position_[19] && position_[19] <= 1800) {
            tracer_state_.position[13] = position_[19];
        } else if (1800 < position_[19] && position_[19] < 3600) {
            tracer_state_.position[13] = position_[19] - 3600;
        }
        // l_wrist_r_joint
        if (0 <= position_[20] && position_[20] <= 1800) {
            tracer_state_.position[14] = position_[20];
        } else if (1800 < position_[20] && position_[20] < 3600) {
            tracer_state_.position[14] = position_[20] - 3600;
        }
        // l_wrist_p_joint
        if (0 <= position_[21] && position_[21] <= 1800) {
            tracer_state_.position[15] = position_[21];
        } else if (1800 < position_[21] && position_[21] < 3600) {
            tracer_state_.position[15] = position_[21] - 3600;
        }
        // // l_thumb_joint(試作1暫定対応)
        // if (0 <= position_[22] && position_[22] <= 110) {
        //     tracer_state_.position[16] = - position_[22];
        // } else if (3150 <= position_[22] && position_[22] < 3600) {
        //     tracer_state_.position[16] = 3600 - position_[22];
        // } else if (1800 < position_[22] && position_[22] < 3150) {
        //     tracer_state_.position[16] = 450;
        // } else if (110 < position_[22] && position_[22] <= 1800) {
        //     tracer_state_.position[16] = -110;
        // }
        // l_thumb_joint(試作2暫定対応)
        if (0 <= position_[22] && position_[22] <= 900) {
            tracer_state_.position[16] = position_[22] - 450;
        } else if (900 < position_[22] && position_[22] <= 1800) {
            tracer_state_.position[16] = 450;
        } else if (1800 < position_[22] && position_[22] < 3600) {
            tracer_state_.position[16] = -450;
        }

        // std::cout << "--------Joy Data--------" << std::endl;
        // for(int i=53;i<59;++i) {
        //     std::cout << "data["<<i<<"] = " << static_cast<int>(tracer_data_[i])<< std::endl;
        // }
        // std::cout << "-------------------------------" << std::endl;


        //////joy sticks & buttons//////
        if (tracer_data_[55] > 122 && tracer_data_[55] < 132 ) {
            tracer_data_[55] = 127;
        }
        if (tracer_data_[56] > 122 && tracer_data_[56] < 132 ) {
            tracer_data_[56] = 127;
        }
        if (tracer_data_[57] > 122 && tracer_data_[57] < 132 ) {
            tracer_data_[57] = 127;
        }
        if (tracer_data_[58] > 122 && tracer_data_[58] < 132 ) {
            tracer_data_[58] = 127;
        }

        // //left joy_stick
        // joy_.axes[0] = static_cast<float>(127 - tracer_data_[55]) / 127;
        // joy_.axes[1] = static_cast<float>(127 - tracer_data_[56]) / 127;

        // // left joy_stick(試作1暫定対応)
        // joy_.axes[1] = static_cast<float>(tracer_data_[55] - 127) / 127;
        // joy_.axes[0] = static_cast<float>(tracer_data_[56] - 127) / 127;

        // left joy_stick(試作2暫定対応)
        joy_.axes[1] = static_cast<float>(127 - tracer_data_[55]) / 127;
        joy_.axes[0] = static_cast<float>(127 - tracer_data_[56]) / 127;

        // //right joy_stick
        // joy_.axes[3] = static_cast<float>(127 - tracer_data_[57]) / 127;
        // joy_.axes[2] = static_cast<float>(127 - tracer_data_[58]) / 127;

        // // right joy_stick(試作1暫定対応)
        // joy_.axes[2] = static_cast<float>(tracer_data_[57] - 127) / 127;
        // joy_.axes[3] = static_cast<float>(tracer_data_[58] - 127) / 127;

        //right joy_stick(試作2暫定対応)
        joy_.axes[2] = static_cast<float>(127 - tracer_data_[57]) / 127;
        joy_.axes[3] = static_cast<float>(127 - tracer_data_[58]) / 127;

        // （foot pedal 有対応）
        if (is_mover_mode) {
            joy_.axes[4] = 1;
            joy_.axes[5] = 0;
        } else {
            joy_.axes[4] = 0;
            joy_.axes[5] = 0;
        }
        if (on_tracer_mode) {
            joy_.buttons[3] = 1;
        } else {
            joy_.buttons[3] = 0;
        }
        joy_.buttons[0] = 0;

        std::string foot_val;
        getFootPedalInput(foot_val);
        // std::cout << "latest_foot_pedal_input_ : "<<foot_val << std::endl;
        if (foot_val == "a") {
            if (is_mover_mode) {
                joy_.axes[4] = 0;
                is_mover_mode = false;
            } else {
                joy_.axes[4] = 1;
                is_mover_mode = true;
            }
            notifyTracerMode();
        } else if (foot_val == "b") {
            if (on_tracer_mode) {
                joy_.buttons[3] = 0;
                on_tracer_mode = false;
            } else {
                joy_.buttons[3] = 1;
                on_tracer_mode = true;
            }
            notifyOnTracer();
        } else if (foot_val == "c") {
            joy_.buttons[0] = 1;
            init_counter_ = 20;
        }

        // （foot pedal 有対応）
        //Left Hand
        switch (tracer_data_[53]) {
            case 32:
                joy_.buttons[4] = 0;
                joy_.buttons[5] = 1;
                break;
            case 128:
                joy_.buttons[4] = 1;
                joy_.buttons[5] = 0;
                break;
            case 160:
                joy_.buttons[4] = 0;
                joy_.buttons[5] = 0;
                break;
            default:
                joy_.buttons[4] = 1;
                joy_.buttons[5] = 1;
                break;
        }

        //Right Hand
        switch (tracer_data_[54]) {
            case 32:
                joy_.buttons[1] = 1;
                joy_.buttons[2] = 0;
                break;
            case 128:
                joy_.buttons[1] = 0;
                joy_.buttons[2] = 1;
                break;
            case 160:
                joy_.buttons[1] = 0;
                joy_.buttons[2] = 0;
                break;
            default:
                joy_.buttons[1] = 1;
                joy_.buttons[2] = 1;
                break;
        }

        // （foot pedal 無対応）
        // //Left Hand
        // switch (tracer_data_[53]) {
        //     case 32:
        //         joy_.axes[4] = -1;
        //         joy_.axes[5] = 0;
        //         break;
        //     case 128:
        //         joy_.axes[4] = 1;
        //         joy_.axes[5] = 0;
        //         break;
        //     case 160:
        //         joy_.axes[4] = 0;
        //         joy_.axes[5] = 0;
        //         break;
        //     default:
        //         joy_.axes[4] = 2; //左右同時押し。elecomパッドだと存在しない
        //         joy_.axes[5] = 0;
        //         break;
        // }

        // //Right Hand
        // switch (tracer_data_[54]) {
        //     case 32:
        //         joy_.buttons[0] = 0;
        //         joy_.buttons[1] = 0;
        //         joy_.buttons[2] = 0;
        //         joy_.buttons[3] = 1;
        //         break;
        //     case 128:
        //         joy_.buttons[0] = 0;
        //         joy_.buttons[1] = 0;
        //         joy_.buttons[2] = 0;
        //         joy_.buttons[3] = 1;
        //         break;
        //     case 160: //(試作2暫定対応)
        //         joy_.buttons[0] = 0;
        //         joy_.buttons[1] = 0;
        //         joy_.buttons[2] = 0;
        //         joy_.buttons[3] = 0;
        //         break;
        //     // case 160: //(試作1暫定対応)
        //     //     joy_.buttons[0] = 0;
        //     //     joy_.buttons[1] = 0;
        //     //     joy_.buttons[2] = 0;
        //     //     joy_.buttons[3] = 1;
        //     //     break;
        //     default:
        //         joy_.buttons[0] = 1;
        //         joy_.buttons[1] = 0;
        //         joy_.buttons[2] = 0;
        //         joy_.buttons[3] = 1;
        //         break;
        // }

        if ((std::abs(joy_.axes[0]) > 0.05 || std::abs(joy_.axes[1]) > 0.05 || std::abs(joy_.axes[3]) > 0.05 || std::abs(joy_.axes[2]) > 0.05) && joy_.axes[4] == 1) {
            joy_.buttons[6] = 1;
            wheel_stop_flag_ = false;
        } else {
            joy_.buttons[6] = 0;
        }

        //for initial pose
        if (init_counter_ > 0) {
            joy_.buttons[0] = 1;
            init_counter_ -=1;
        }

        if (joy_.buttons[3] == 1) {
            tracer_state_.header.stamp = this->now();
            tracer_state_pub_->publish(tracer_state_);
            joy_pub_->publish(joy_);
        }

        //dummy cmd_vel publish
        if (joy_.buttons[6] == 0 && wheel_stop_flag_ == false) {
            cmd_vel_.linear.x = 0;
            cmd_vel_.linear.y = 0;
            cmd_vel_.linear.z = 0;
            cmd_vel_.angular.x = 0;
            cmd_vel_.angular.y = 0;
            cmd_vel_.angular.z= 0;
            cmd_vel_pub_->publish(cmd_vel_);
            wheel_stop_flag_ = true;
        }

        if (tracer_mode_ == 1 && joy_.buttons[3] == 0 ) {
            if (initial_pose_ == "zero") {
                init_counter_ = 20;
                tracer_state_.position[0] = 0;

                //right arm
                tracer_state_.position[1] = 0;
                tracer_state_.position[2] = 0;
                tracer_state_.position[3] = 0;
                tracer_state_.position[4] = 0;
                tracer_state_.position[5] = 0;
                tracer_state_.position[6] = 0;
                tracer_state_.position[7] = 0;
                tracer_state_.position[8] = 0;

                //left arm
                tracer_state_.position[9] = 0;
                tracer_state_.position[10] = 0;
                tracer_state_.position[11] = 0;
                tracer_state_.position[12] = 0;
                tracer_state_.position[13] = 0;
                tracer_state_.position[14] = 0;
                tracer_state_.position[15] = 0;
                tracer_state_.position[16] = 0;
                tracer_state_.header.stamp = this->now();
                tracer_state_pub_->publish(tracer_state_);
                std::cout << "go to initial pose" << std::endl;
            }
        }
        tracer_mode_ = joy_.buttons[3];
    }
}

void TracerTeleop::updateFootPedalInput() {
    if (!foot_pedal_enabled || foot_pedal_ == nullptr) {
        return;
    }

    std::string input;
    while (foot_pedal_->get_input(input)) {
        {
            std::lock_guard<std::mutex> lock(foot_pedal_input_mutex_);
            latest_foot_pedal_input_ = input;
        }
    }
}

bool TracerTeleop::getFootPedalInput(std::string& input) {
    std::lock_guard<std::mutex> lock(foot_pedal_input_mutex_);
    input = latest_foot_pedal_input_;
    latest_foot_pedal_input_.clear();
    return true;
}

void TracerTeleop::notifyOnTracer() {
    std_msgs::msg::Bool msg;
    msg.data = on_tracer_mode;
    on_tracer_pub_->publish(msg);
}

void TracerTeleop::notifyTracerMode() {
    std_msgs::msg::Bool msg;
    msg.data = is_mover_mode;
    tracer_mode_pub_->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TracerTeleop>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
