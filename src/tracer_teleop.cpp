#include "motion_tracer_ros2/tracer_teleop.hpp"

TracerTeleop::TracerTeleop() :
    Node("tracer_teleop_node"),init_counter_(0),tracer_mode_(0) {

    tracer_ = new tracer::controller::TracerCommand();

    if (!tracer_->port_open("/dev/tracer_usb", 460800)) {
    // if (!tracer_->port_open("/dev/tracer_usb", 1000000)) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed");
        rclcpp::shutdown();
        return;
    }

    this->declare_parameter<std::string>("initial_pose", "");
    initial_pose_ = this->get_parameter("initial_pose").as_string();

    // Publisher
    tracer_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("tracer_states", 1);
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("tracer_joy", 1);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 1);

    // Subscriber
    current_sub_ = this->create_subscription<aero_controller_msgs::msg::Current>("/current_controller/current", 1, std::bind(&TracerTeleop::currentCallback, this, std::placeholders::_1));

    // JointState初期化
    tracer_state_.name.resize(17);
    tracer_state_.position.resize(17);

    tracer_state_.header.frame_id ="tracer_base";

    joy_.axes.resize(6, 0.0);
    joy_.buttons.resize(13, 0);

    position_.resize(30, 0);

    latest_current_.resize(60, 0);

    wheel_stop_flag_ = true;

    timer_ = this->create_wall_timer(std::chrono::milliseconds(20),std::bind(&TracerTeleop::processLoop, this));

    running_ = true;
    tx_thread_ = std::thread(&TracerTeleop::txLoop, this);
}

TracerTeleop::~TracerTeleop() {
    delete tracer_;
    running_ = false;

    if(tx_thread_.joinable())
    {
        tx_thread_.join();
    }
}

void TracerTeleop::currentCallback(const aero_controller_msgs::msg::Current& _current_data) {
    std::lock_guard<std::mutex> lock(current_mutex_);
    for (unsigned int idx = 0; idx < _current_data.data.size(); ++idx) {
        latest_current_[idx] = _current_data.data[idx];
    }
}

void TracerTeleop::txLoop() {

    rclcpp::WallRate rate(1000.0);

    std::vector<uint8_t> current_copy;
    while(running_) {
        {
            std::lock_guard<std::mutex> lock(current_mutex_);
            current_copy = latest_current_;
        }

        if(current_copy.size()!= 60){
            return;
        }

        std::vector<uint16_t> currents(current_copy.size()/2);

        // 初期化
        for (unsigned int idx = 0; idx < currents.size(); ++idx) {
            currents[idx] = 0x7FFF;
        }

        if (only_hand_current) {
            int right_hand_aero_id = 11;
            int left_hand_aero_id = 26;
            double scale = 1.0;

            uint16_t r_hand_current = static_cast<uint16_t>(current_copy[right_hand_aero_id * 2]) << 8 |
                                        static_cast<uint16_t>(current_copy[right_hand_aero_id * 2 + 1]);
            uint16_t l_hand_current = static_cast<uint16_t>(current_copy[left_hand_aero_id * 2]) << 8 |
                                        static_cast<uint16_t>(current_copy[left_hand_aero_id * 2 + 1]);

            currents[16] = r_hand_current * scale;
            currents[8] = l_hand_current * scale;
            
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

void TracerTeleop::processPacket(const std::vector<uint8_t>& tracer_data_) {
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

        std::cout << "--------Tracer Position--------" << std::endl;
        for(int i=0;i<30;++i) {
            std::cout << "position_["<<i<<"] = " << position_[i]<< std::endl;
        }
        std::cout << "-------------------------------" << std::endl;

        // waist
        if (-900 <= position_[0] && position_[0] <= 900) {
            tracer_state_.position[0] = position_[0] * 5;
        }

        // ######right arm######
        // r_shoulder_p_joint
        if (0 <= position_[9] && position_[9] <= 986) {
            tracer_state_.position[1] = position_[9];
        } else if (986 < position_[9] && position_[9] <= 1800) {
            tracer_state_.position[1] = 986;
        }
        // r_shoulder_r_joint
        if (0 <= position_[10] && position_[10] <= 900) {
            tracer_state_.position[2] = position_[10];
        } else if (900 < position_[10] && position_[10] < 1800) {
            tracer_state_.position[2] = 900;
        }
        // r_shoulder_y_joint
        if (0 <= position_[11] && position_[11] <= 1800) {
            tracer_state_.position[3] = position_[11];
        } else if (1800 < position_[11] && position_[11] < 3600) {
            tracer_state_.position[3] = position_[11] - 3600;
        }
        // r_elbow_joint
        if (1880 <= position_[12] && position_[12] < 3600) {
            tracer_state_.position[4] = position_[12] - 3600;
        } else if (900 < position_[12] && position_[12] < 1880) {
            tracer_state_.position[4] = -1720;
        }
        // r_wrist_y_joint
        if (0 <= position_[13] && position_[13] <= 1800) {
            tracer_state_.position[5] = position_[13];
        } else if (1800 < position_[13] && position_[13] < 3600) {
            tracer_state_.position[5] = position_[13] - 3600;
        }
        // r_wrist_r_joint
        if (0 <= position_[14] && position_[14] <= 1800) {
            tracer_state_.position[6] = position_[14];
        } else if (1800 < position_[14] && position_[14] < 3600) {
            tracer_state_.position[6] = position_[14] - 3600;
        }
        // r_wrist_p_joint
        if (0 <= position_[15] && position_[15] <= 1800) {
            tracer_state_.position[7] = position_[15];
        } else if (1800 < position_[15] && position_[15] < 3600) {
            tracer_state_.position[7] = position_[15] - 3600;
        }
        // r_thumb_joint
        if (0 <= position_[16] && position_[16] <= 110) {
            tracer_state_.position[8] = position_[16];
        } else if (3150 < position_[16] && position_[16] < 3600) {
            tracer_state_.position[8] = position_[16] - 3600;
        }

        // ######left arm######
        // l_shoulder_p_joint
        if (2614 <= position_[1] && position_[1] < 3600) {
            tracer_state_.position[9] = position_[1] - 3600;
        } else if (1800 <= position_[1] && position_[1] < 2614) {
            tracer_state_.position[9] = -986;
        }
        // l_shoulder_r_joint
        if (2700 <= position_[2] && position_[2] < 3600) {
            tracer_state_.position[10] = position_[2] - 3600;
        } else if (1800 <= position_[2] && position_[2] < 2700) {
            tracer_state_.position[10] = -900;
        }
        // l_shoulder_y_joint
        if (0 <= position_[3] && position_[3] <= 1800) {
            tracer_state_.position[11] = position_[3];
        } else if (1800 < position_[3] && position_[3] < 3600) {
            tracer_state_.position[11] = position_[3] - 3600;
        }
        // l_elbow_joint
        if (0 <= position_[4] && position_[4] <= 1720) {
            tracer_state_.position[12] = position_[4];
        } else if (1720 < position_[4] && position_[4] < 2700) {
            tracer_state_.position[12] = 1720;
        }
        // l_wrist_y_joint
        if (0 <= position_[5] && position_[5] <= 1800) {
            tracer_state_.position[13] = position_[5];
        } else if (1800 < position_[5] && position_[5] < 3600) {
            tracer_state_.position[13] = position_[5] - 3600;
        }
        // l_wrist_r_joint
        if (0 <= position_[6] && position_[6] <= 1800) {
            tracer_state_.position[14] = position_[6];
        } else if (1800 < position_[6] && position_[6] < 3600) {
            tracer_state_.position[14] = position_[6] - 3600;
        }
        // l_wrist_p_joint
        if (0 <= position_[7] && position_[7] <= 1800) {
            tracer_state_.position[15] = position_[7];
        } else if (1800 < position_[7] && position_[7] < 3600) {
            tracer_state_.position[15] = position_[7] - 3600;
        }
        // l_thumb_joint
        if (0 <= position_[8] && position_[8] < 450) {
            tracer_state_.position[16] = position_[8];
        } else if (3490 < position_[8] && position_[8] < 3600) {
            tracer_state_.position[16] = position_[8] - 3600;
        }

        tracer_state_.header.stamp = this->now();
        tracer_state_pub_->publish(tracer_state_);

    } else if (tracer_data_[0] == 0xFB) {
        if (tracer_data_.size() < 7) {
            return;
        }

        std::vector<uint8_t> tr_data_ = tracer_data_;
        if (tr_data_[6] > 122 && tr_data_[6] < 132 ) {
            tr_data_[6] = 127;
        }
        if (tr_data_[7] > 122 && tr_data_[7] < 132 ) {
            tr_data_[7] = 127;
        }
        if (tr_data_[8] > 122 && tr_data_[8] < 132 ) {
            tr_data_[8] = 127;
        }
        if (tr_data_[9] > 122 && tr_data_[9] < 132 ) {
            tr_data_[9] = 127;
        }

        //left joy_stick
        joy_.axes[0] = static_cast<float>(127 - tr_data_[6]) / 127;
        joy_.axes[1] = static_cast<float>(127 - tr_data_[7]) / 127;

        //right joy_stick
        joy_.axes[3] = static_cast<float>(127 - tr_data_[8]) / 127;
        joy_.axes[2] = static_cast<float>(127 - tr_data_[9]) / 127;

        //Left Hand
        switch (tr_data_[4]) {
            case 32:
                joy_.axes[4] = -1;
                joy_.axes[5] = 0;
                break;
            case 128:
                joy_.axes[4] = 1;
                joy_.axes[5] = 0;
                break;
            case 160:
                joy_.axes[4] = 2; //左右同時押し。elecomパッドだと存在しない
                joy_.axes[5] = 0;
                break;
            default:
                joy_.axes[4] = 0;
                joy_.axes[5] = 0;
                break;

        }

        //Right Hand
        switch (tr_data_[5]) {
            case 32:
                joy_.buttons[0] = 0;
                joy_.buttons[1] = 0;
                joy_.buttons[2] = 0;
                joy_.buttons[3] = 1;
                break;
            case 128:
                joy_.buttons[0] = 1;
                joy_.buttons[1] = 0;
                joy_.buttons[2] = 0;
                joy_.buttons[3] = 0;
                break;
            case 160:
                joy_.buttons[0] = 1;
                joy_.buttons[1] = 0;
                joy_.buttons[2] = 0;
                joy_.buttons[3] = 1;
                break;
            default:
                joy_.buttons[0] = 0;
                joy_.buttons[1] = 0;
                joy_.buttons[2] = 0;
                joy_.buttons[3] = 0;
                break;
        }

        if ((std::abs(joy_.axes[0]) > 0.05 || std::abs(joy_.axes[1]) > 0.05 || std::abs(joy_.axes[3]) > 0.05 || std::abs(joy_.axes[2]) > 0.05) && joy_.axes[4] == 1) {
            joy_.buttons[6] = 1;
            wheel_stop_flag_ = false;
        } else {
            joy_.buttons[6] = 0;
        }

        //for initial pose
        if (init_counter_ > 0) {
            joy_.axes[4] = -1;
            joy_.buttons[0] = 1;
            init_counter_ -=1;
        }

        if (joy_.buttons[3] == 1) {
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
            if (initial_pose_ == "MC") {
                init_counter_ = 50;
                tracer_state_.position[0] = 0;
                //right arm
                tracer_state_.position[1] = -200;
                tracer_state_.position[2] = 100;
                tracer_state_.position[3] = -135;
                tracer_state_.position[4] = 800;
                tracer_state_.position[5] = 0;
                tracer_state_.position[6] = 100;
                tracer_state_.position[7] = 150;
                tracer_state_.position[8] = 1700;

                //left arm
                tracer_state_.position[9] = -200;
                tracer_state_.position[10] = 100;
                tracer_state_.position[11] = -135;
                tracer_state_.position[12] = 800;
                tracer_state_.position[13] = 0;
                tracer_state_.position[14] = 100;
                tracer_state_.position[15] = 150;
                tracer_state_.position[16] = 1700;
                tracer_state_.header.stamp = this->get_clock()->now();
                tracer_state_pub_->publish(tracer_state_);
                std::cout << "go to initial pose" << std::endl;
            } else if (initial_pose_ == "zero") {
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
                tracer_state_.position[8] = 300;

                //left arm
                tracer_state_.position[9] = 0;
                tracer_state_.position[10] = 0;
                tracer_state_.position[11] = 0;
                tracer_state_.position[12] = 0;
                tracer_state_.position[13] = 0;
                tracer_state_.position[14] = 0;
                tracer_state_.position[15] = 0;
                tracer_state_.position[16] = 1000;
                tracer_state_.header.stamp = this->get_clock()->now();
                tracer_state_pub_->publish(tracer_state_);
                std::cout << "go to initial pose" << std::endl;
            }
        }
        tracer_mode_ = joy_.buttons[3];
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TracerTeleop>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}