#include "motion_tracer_ros2/wrench_controller.hpp"

WrenchController::WrenchController() : 
    Node("wrench_controller_node") {

    configureArm(
        right_arm_, "right", "r",
        "/dev/force_sensor_right", "r_arm_link", "r_hand_link",
        {0.0, 0.0, -0.011}, {M_PI, 0.0, -M_PI / 2.0},
        {
            "r_shoulder_p_joint",
            "r_shoulder_r_joint",
            "r_shoulder_y_joint",
            "r_elbow_joint",
            "r_wrist_y_joint",
            "r_wrist_r_joint",
            "r_wrist_p_joint"
        },
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
        {0, 1, 2, 3, 4, 5, 6},
        {0.0, 0.0, 0.05}, {0.0, 0.0, -9.80665}, 1.0
    );

    configureArm(
        left_arm_, "left", "l",
        "/dev/force_sensor_left", "l_arm_link", "l_hand_link",
        {0.0, 0.0, -0.011}, {M_PI, 0.0, -M_PI / 2.0},
        {
            "l_shoulder_p_joint",
            "l_shoulder_r_joint",
            "l_shoulder_y_joint",
            "l_elbow_joint",
            "l_wrist_y_joint",
            "l_wrist_r_joint",
            "l_wrist_p_joint"
        },
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
        {15, 16, 17, 18, 19, 20, 21},
        {0.0, 0.0, 0.05}, {0.0, 0.0, -9.80665}, 1.0
    );

    if (!validateArmConfiguration(right_arm_) || !validateArmConfiguration(left_arm_)) {
        throw std::runtime_error("Invalid dual-arm force-reflection configuration");
    }

    auto current_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto notify_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto robot_description_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    current_pub_ = this->create_publisher<aero_controller_msgs::msg::Current>("/current_controller/current", current_qos);
    current_raw_sub_ = this->create_subscription<aero_controller_msgs::msg::Current>("/current_controller/current_raw", current_qos, std::bind(&WrenchController::currentRawCallback, this, std::placeholders::_1));
    robot_description_sub_ = create_subscription<std_msgs::msg::String>("/robot_description", robot_description_qos, std::bind(&WrenchController::robotDescriptionCallback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", sensor_qos, std::bind(&WrenchController::jointStateCallback, this, std::placeholders::_1));
    on_force_feedback_sub_ = create_subscription<std_msgs::msg::Bool>("/on_force_feedback", notify_qos, std::bind(&WrenchController::onForceFeedbackCallback, this, std::placeholders::_1));
    if (publish_wrench_debug) {
        right_arm_.wrench_pub = create_publisher<geometry_msgs::msg::WrenchStamped>("/force_sensor/right/filtered_wrench", sensor_qos);
        left_arm_.wrench_pub = create_publisher<geometry_msgs::msg::WrenchStamped>("/force_sensor/left/filtered_wrench", sensor_qos);
        right_arm_.dynamics_compensated_wrench_pub = create_publisher<geometry_msgs::msg::WrenchStamped>("/force_sensor/right/dynamics_compensated_wrench", sensor_qos);
        left_arm_.dynamics_compensated_wrench_pub = create_publisher<geometry_msgs::msg::WrenchStamped>("/force_sensor/left/dynamics_compensated_wrench", sensor_qos);
    }
    if (publish_tau_debug) {
        right_arm_.tau_pub = create_publisher<sensor_msgs::msg::JointState>("/force_sensor/right/reflection_tau", sensor_qos);
        left_arm_.tau_pub = create_publisher<sensor_msgs::msg::JointState>("/force_sensor/left/reflection_tau", sensor_qos);
    }
    tof_raw_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/tof_sensor/scan_raw", sensor_qos);

    tare_srv_ = this->create_service<std_srvs::srv::Trigger>("/force_sensor/tare", std::bind(&WrenchController::tareCallback, this, std::placeholders::_1, std::placeholders::_2));

    right_arm_.last_current_publish_time = now();
    left_arm_.last_current_publish_time = now();

    if (!initializeSensor(right_arm_)) {
        RCLCPP_ERROR(this->get_logger(), "Right force sensor initialization failed: %s", right_arm_.device.c_str());
    }
    if (!initializeSensor(left_arm_)) {
        RCLCPP_ERROR(this->get_logger(), "Left force sensor initialization failed: %s", left_arm_.device.c_str());
    }

    publishNeutralCurrent();
    running_.store(true);
    wrench_thread_ = std::thread(&WrenchController::wrenchLoop, this);
    tof_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&WrenchController::publishToFLoop, this));
}

WrenchController::~WrenchController() {
    running_.store(false);
    if (wrench_thread_.joinable()) {
        wrench_thread_.join();
    }

    stopStream(right_arm_);
    closeSerial(right_arm_);
    stopStream(left_arm_);
    closeSerial(left_arm_);

    publishNeutralCurrent();
}

void WrenchController::configureArm(
    ArmContext & arm, const std::string & side, const std::string & joint_prefix,
    const std::string & default_device, const std::string & default_base_link, const std::string & default_tip_link,
    const std::vector<double> & default_sensor_xyz_in_tip, const std::vector<double> & default_sensor_rpy_in_tip,
    const std::vector<std::string> & default_active_joint_names,
    const std::vector<double> & default_current_signs, const std::vector<double> & default_current_gains,
    const std::vector<int64_t> & default_arm_indices,
    const std::vector<double> & default_payload_com_in_sensor, const std::vector<double> & default_gravity_vector_in_base,
    const double & default_gravity_compensation_sign) {

    arm.side = side;
    arm.joint_prefix = joint_prefix;
    arm.device = default_device;
    arm.sensor_frame_id = joint_prefix + "_force_sensor";
    arm.base_link = default_base_link;
    arm.tip_link = default_tip_link;
    arm.sensor_xyz_in_tip = default_sensor_xyz_in_tip;
    arm.sensor_rpy_in_tip = default_sensor_rpy_in_tip;
    arm.active_joint_names = default_active_joint_names;
    arm.dynamics_active_joint_names = {
        "waist_y_joint",
        "waist_p_joint",
        "waist_r_joint"
    };
    arm.dynamics_active_joint_names.insert(arm.dynamics_active_joint_names.end(), arm.active_joint_names.begin(), arm.active_joint_names.end());
    arm.current_signs = default_current_signs;
    arm.current_gains = default_current_gains;
    arm.arm_joint_indices = default_arm_indices;
    arm.payload_com_in_sensor = default_payload_com_in_sensor;
    arm.gravity_vector_in_base = default_gravity_vector_in_base;
    arm.gravity_compensation_sign = default_gravity_compensation_sign;
    arm.last_current_delta.assign(arm.active_joint_names.size(), 0.0);
    arm.tip_T_sensor = makeSensorFrameFromParams(arm);
}

bool WrenchController::validateArmConfiguration(const ArmContext & arm) const {

    if (arm.sensor_xyz_in_tip.size() != 3 || arm.sensor_rpy_in_tip.size() != 3) {
        RCLCPP_ERROR(this->get_logger(), "%s sensor xyz/rpy parameters must each have three values.", arm.side.c_str());
        return false;
    }

    if (arm.payload_com_in_sensor.size() != 3 || arm.payload_inertia_diag_in_sensor.size() != 3 || arm.gravity_vector_in_base.size() != 3) {
        RCLCPP_ERROR(this->get_logger(), "%s payload COM, inertia diagonal, and gravity vectors must each have three values.", arm.side.c_str());
        return false;
    }
    if (arm.payload_mass_kg < 0.0) {
        RCLCPP_ERROR(this->get_logger(), "%s payload_mass_kg must be non-negative.", arm.side.c_str());
        return false;
    }

    const size_t n = arm.active_joint_names.size();
    if (arm.dynamics_base_link.empty() || arm.dynamics_active_joint_names.size() != n + 3) {
        RCLCPP_ERROR(this->get_logger(), "%s dynamics chain must contain waist_y/p/r plus seven arm joints.", arm.side.c_str());
        return false;
    }
    if (n != 7 || arm.arm_joint_indices.size() != n || arm.current_signs.size() != n || arm.current_gains.size() != n) {
        RCLCPP_ERROR(this->get_logger(), "%s arm parameter sizes must all be seven: active=%zu indices=%zu signs=%zu gains=%zu", arm.side.c_str(), n, arm.arm_joint_indices.size(), arm.current_signs.size(), arm.current_gains.size());
        return false;
    }

    for (const auto index : arm.arm_joint_indices) {
        if (index < 0 || index >= MOTOR_COUNT || isProtectedTracerIndex(static_cast<int>(index))) {
            RCLCPP_ERROR(this->get_logger(), "%s arm has invalid or protected Tracer index: %ld", arm.side.c_str(), static_cast<long>(index));
            return false;
        }
    }
    return true;
}

void WrenchController::currentRawCallback(const aero_controller_msgs::msg::Current::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(current_raw_mutex);
    latest_current_raw = *msg;
    latest_current_raw_stamp = this->now();
    have_current_raw.store(true);
}

void WrenchController::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!msg || msg->data.empty()) {
        return;
    }
    if (!right_arm_.kinematics_ready.load()) {
        buildKinematicsFromUrdfForArm(right_arm_, msg->data);
    }
    if (!left_arm_.kinematics_ready.load()) {
        buildKinematicsFromUrdfForArm(left_arm_, msg->data);
    }
}

void WrenchController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg) {
        return;
    }
    std::lock_guard<std::mutex> lock(joint_mutex);
    const size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i) {
        latest_joint_position[msg->name[i]] = msg->position[i];
    }
    latest_joint_velocity.clear();
    const size_t nv = std::min(msg->name.size(), msg->velocity.size());
    for (size_t i = 0; i < nv; ++i) {
        latest_joint_velocity[msg->name[i]] = msg->velocity[i];
    }
    latest_joint_stamp = msg->header.stamp;
    if (latest_joint_stamp.nanoseconds() == 0) {
        latest_joint_stamp = now();
    }
    have_joint_state.store(true);
}

void WrenchController::onForceFeedbackCallback(const std_msgs::msg::Bool & msg) {
    on_force_feedback_.store(msg.data);
    if (!msg.data) {
        right_arm_.reflection_state_reset_requested.store(true);
        left_arm_.reflection_state_reset_requested.store(true);
    }
}

void WrenchController::tareCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    right_arm_.tare_requested.store(true);
    left_arm_.tare_requested.store(true);
    right_arm_.filter_initialized.store(false);
    left_arm_.filter_initialized.store(false);
    right_arm_.payload_reference_valid.store(false);
    left_arm_.payload_reference_valid.store(false);
    right_arm_.payload_reference_pending.store(true);
    left_arm_.payload_reference_pending.store(true);
    right_arm_.dynamics_state_reset_requested.store(true);
    left_arm_.dynamics_state_reset_requested.store(true);
    right_arm_.reflection_state_reset_requested.store(true);
    left_arm_.reflection_state_reset_requested.store(true);
    res->success = true;
    res->message = "force sensor tare requested.";
}

bool WrenchController::initializeSensor(ArmContext & arm) {
    if (!openSerial(arm)) {
        return false;
    }

    bool calibrated = false;
    for (size_t i = 0; i < retry_calib_max_count; ++i) {
        if (readCalibration(arm)) {
            calibrated = true;
            break;
        }
        RCLCPP_WARN(get_logger(), "%s force sensor calibration read failed (%zu/%zu).", arm.side.c_str(), i + 1, retry_calib_max_count);
    }

    if (!calibrated) {
        RCLCPP_ERROR(get_logger(), "%s force sensor calibration could not be read.", arm.side.c_str());
        return false;
    }

    if (arm.acquisition_mode == "continuous" && !startStream(arm)) {
        return false;
    }
    arm.sensor_ready.store(true);
    return true;
}

bool WrenchController::openSerial(ArmContext & arm) {
    std::lock_guard<std::mutex> lock(arm.serial_mutex);
    arm.fd = open(arm.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (arm.fd < 0) {
        return false;
    }

    struct termios term;
    tcgetattr(arm.fd, &term);
    memset(&term, 0, sizeof(term));
    term.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
    term.c_iflag = IGNPAR;
    term.c_oflag = 0;
    term.c_lflag = 0;

    term.c_cc[VMIN]  = 1;
    term.c_cc[VTIME] = 0;

    term.c_cc[VINTR]    = 0;
    term.c_cc[VQUIT]    = 0;
    term.c_cc[VERASE]   = 0;
    term.c_cc[VKILL]    = 0;
    term.c_cc[VEOF]     = 4;
    term.c_cc[VSWTC]    = 0;
    term.c_cc[VSTART]   = 0;
    term.c_cc[VSTOP]    = 0;
    term.c_cc[VSUSP]    = 0;
    term.c_cc[VEOL]     = 0;
    term.c_cc[VREPRINT] = 0;
    term.c_cc[VDISCARD] = 0;
    term.c_cc[VWERASE]  = 0;
    term.c_cc[VLNEXT]   = 0;
    term.c_cc[VEOL2]    = 0;

    tcsetattr(arm.fd, TCSANOW, &term);

    return true;
}

void WrenchController::closeSerial(ArmContext & arm) {
    std::lock_guard<std::mutex> lock(arm.serial_mutex);
    if (arm.fd >= 0) {
        close(arm.fd);
        arm.fd = -1;
    }
}

void WrenchController::flushSerial(ArmContext & arm) {
    arm.continuous_rx_buffer.clear();
    if (arm.fd >= 0) {
        tcflush(arm.fd, TCIFLUSH);
    }
}

bool WrenchController::writeAll(ArmContext & arm, const char * data, size_t len) {
    size_t written = 0;
    while (written < len) {
        const ssize_t n = write(arm.fd, data + written, len - written);
        if (n > 0) {
            written += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            continue;
        }
        return false;
    }
    if (arm.fd >= 0) {
        tcdrain(arm.fd);
    }
    return true;
}

bool WrenchController::readCalibText(ArmContext & arm, std::string& line, int timeout_ms, size_t max_len) {
    line.clear();

    if (arm.fd < 0) {
        return false;
    }

    const auto start = std::chrono::steady_clock::now();
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        const int remaining_ms = timeout_ms - static_cast<int>(elapsed_ms);
        if (remaining_ms <= 0) {
            return false;
        }

        pollfd pfd;
        pfd.fd = arm.fd;
        pfd.events = POLLIN;
        const int poll_ret = poll(&pfd, 1, remaining_ms);
        if (poll_ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (poll_ret == 0) {
            return false;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            return false;
        }
        if (!(pfd.revents & POLLIN)) {
            continue;
        }

        char ch = 0;
        const ssize_t n = read(arm.fd, &ch, 1);
        if (n == 1) {
            if (ch == '\n') {
                return true;
            }
            if (ch == '\r') {
                continue;
            }
            line.push_back(ch);
            if (line.size() > max_len) {
                return false;
            }
            continue;
        }

        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;
        }
        return false;
    }
}

bool WrenchController::readCalibration(ArmContext & arm) {
    std::lock_guard<std::mutex> lock(arm.serial_mutex);
    if (arm.fd < 0) {
        return false;
    }
    flushSerial(arm);

    if (!writeAll(arm, "E", 1)) {
        RCLCPP_WARN(this->get_logger(), "Failed to send stop stream command.");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!writeAll(arm ,"p", 1)) {
        return false;
    }

    std::string reply;
    if (!readCalibText(arm, reply, 100, 128)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read force sensor calibration reply.");
        return false;
    }

    double c[6];
    const int n = std::sscanf(reply.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf", &c[0], &c[1], &c[2], &c[3], &c[4], &c[5]);
    if (n != 6) {
        return false;
    }
    for (size_t i = 0; i < arm.calib.size(); ++i) {
        arm.calib[i] = std::max(1e-9, static_cast<double>(c[i]));
    }
    RCLCPP_INFO(this->get_logger(), "%s force sensor calibration: %.3f %.3f %.3f %.3f %.3f %.3f",
                arm.side.c_str(), arm.calib[0], arm.calib[1], arm.calib[2], arm.calib[3], arm.calib[4], arm.calib[5]);
    flushSerial(arm);
    return true;
}

bool WrenchController::startStream(ArmContext & arm) {
    std::lock_guard<std::mutex> lock(arm.serial_mutex);
    return startStreamUnlocked(arm);
}

bool WrenchController::startStreamUnlocked(ArmContext & arm) {
    if (arm.fd < 0) {
        return false;
    }
    if (arm.continuous_output_started.load()) {
        return true;
    }

    flushSerial(arm);
    for (size_t i = 0; i < 3; ++i) {
        if (!writeAll(arm, "O", 1)){
            return false;
        };
    }

    if (!writeAll(arm ,"S", 1)) {
        return false;
    }

    arm.continuous_output_started.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    flushSerial(arm);
    return true;
}

bool WrenchController::stopStream(ArmContext & arm) {
    std::lock_guard<std::mutex> lock(arm.serial_mutex);
    return stopStreamUnlocked(arm);
}

bool WrenchController::stopStreamUnlocked(ArmContext & arm) {
    if (arm.fd < 0) {
        return false;
    }
    if (!arm.continuous_output_started.load()) {
        return true;
    }

    if (!writeAll(arm, "E", 1)) {
        RCLCPP_WARN(this->get_logger(), "Failed to send stop stream command.");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    arm.continuous_output_started.store(false);
    flushSerial(arm);
    return true;
}

bool WrenchController::handleTare(ArmContext & arm) {
    if (arm.fd < 0) {
        return false;
    }

    const bool was_continuous = arm.acquisition_mode == "continuous" && arm.continuous_output_started.load();
    if (was_continuous) {
        stopStreamUnlocked(arm);
    }

    flushSerial(arm);
    for (size_t i = 0; i < 3; ++i) {
        if (!writeAll(arm, "O", 1)){
            if (was_continuous) {
                startStreamUnlocked(arm);
            }
            return false;
        };
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    flushSerial(arm);
    arm.filter_initialized.store(false);

    if (was_continuous) {
        startStreamUnlocked(arm);
    }
    return false;  // Do not use a sample in the same cycle as tare.
}

bool WrenchController::readExact(ArmContext & arm, uint8_t *dst, size_t len, int timeout_ms) {
    if (arm.fd < 0 || dst == nullptr) {
        return false;
    }
    size_t total = 0;
    const auto start = std::chrono::steady_clock::now();
    while (total < len) {
        const auto now_tp = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - start).count();
        const int remaining_ms = timeout_ms - static_cast<int>(elapsed);
        if (remaining_ms < 0) {
            return false;
        }

        pollfd pfd;
        pfd.fd = arm.fd;
        pfd.events = POLLIN;
        const int ret = poll(&pfd, 1, remaining_ms);
        if (ret <= 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            return false;
        }
        if (!(pfd.revents & POLLIN)) {
            continue;
        }

        const ssize_t n = read(arm.fd, dst + total, len - total);
        if (n > 0) {
            total += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;
        }
        return false;
    }
    return true;
}

bool WrenchController::readSampleRequestResponse(ArmContext & arm, Wrench6 & out) {
    std::lock_guard<std::mutex> lock(arm.serial_mutex);
    if (arm.fd < 0) {
        return false;
    }
    if (arm.tare_requested.exchange(false)) {
        return handleTare(arm);
    }

    flushSerial(arm);
    if (!writeAll(arm, "R", 1)) {
        return false;
    }

    std::array<uint8_t, WRENCH_DATA_LENGTH> frame;
    if (!readExact(arm, frame.data(), frame.size(), 2)) {
        flushSerial(arm);
        return false;
    }
    return parseFrame(arm, frame.data(), out);
}

bool WrenchController::extractLatestFrame(ArmContext & arm, std::array<uint8_t, WRENCH_DATA_LENGTH> & latest_frame) {
    bool found = false;
    size_t erase_until = 0;

    for (size_t i = 0; i + 1 < arm.continuous_rx_buffer.size(); ++i) {
        if (arm.continuous_rx_buffer[i] != CR || arm.continuous_rx_buffer[i + 1] != LF) {
            continue;
        }
        if (i >= WRENCH_PAYLOAD_DATA_LENGTH) {
            const size_t frame_start = i - WRENCH_PAYLOAD_DATA_LENGTH;
            if (frame_start + WRENCH_DATA_LENGTH <= arm.continuous_rx_buffer.size()) {
                std::copy_n(arm.continuous_rx_buffer.begin() + frame_start, WRENCH_DATA_LENGTH, latest_frame.begin());
                found = true;
            }
        }
        erase_until = i + 2;
        ++i;
    }

    if (erase_until > 0) {
        arm.continuous_rx_buffer.erase(arm.continuous_rx_buffer.begin(), arm.continuous_rx_buffer.begin() + erase_until);
    }
    return found;
}

bool WrenchController::readStream(ArmContext & arm, int timeout_ms) {
    if (arm.fd < 0) {
        return false;
    }

    pollfd pfd;
    pfd.fd = arm.fd;
    pfd.events = POLLIN;
    const int ret = ::poll(&pfd, 1, timeout_ms);
    if (ret < 0) {
        if (errno == EINTR) {
            return true;
        }
        return false;
    }
    if (ret == 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) || !(pfd.revents & POLLIN)) {
        return false;
    }

    bool got_any = false;
    uint8_t buf[512];
    while (true) {
        const ssize_t n = read(arm.fd, buf, sizeof(buf));
        if (n > 0) {
            got_any = true;
            arm.continuous_rx_buffer.insert(arm.continuous_rx_buffer.end(), buf, buf + n);
            const size_t max_buf = static_cast<size_t>(frame_rx_buffer_max_bytes);
            if (arm.continuous_rx_buffer.size() > max_buf) {
                const size_t keep = static_cast<size_t>(frame_rx_buffer_keep_bytes);
                if (arm.continuous_rx_buffer.size() > keep) {
                    arm.continuous_rx_buffer.erase(arm.continuous_rx_buffer.begin(), arm.continuous_rx_buffer.end() - keep);
                }
            }
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;
        }
        if (n < 0) {
            return false;
        }
        break;
    }
    return got_any;
}

bool WrenchController::readSampleContinuous(ArmContext & arm, Wrench6 & out) {
    std::lock_guard<std::mutex> lock(arm.serial_mutex);
    if (arm.fd < 0) {
        return false;
    }
    if (arm.tare_requested.exchange(false)) {
        return handleTare(arm);
    }
    if (!arm.continuous_output_started.load() && !startStreamUnlocked(arm)) {
        return false;
    }

    if (!readStream(arm, 2)) {
        return false;
    }

    for (int i = 0; i < 10; ++i) {
        if (!readStream(arm, 0)) {
            break;
        }
    }

    std::array<uint8_t, WRENCH_DATA_LENGTH> frame;
    if (!extractLatestFrame(arm, frame)) {
        return false;
    }
    return parseFrame(arm, frame.data(), out);
}

bool WrenchController::readSample(ArmContext & arm, Wrench6 & out) {
    if (arm.acquisition_mode == "continuous") {
        return readSampleContinuous(arm, out);
    }
    return readSampleRequestResponse(arm, out);
}

bool WrenchController::parseFrame(ArmContext & arm, const uint8_t* frame, Wrench6& out) {
    if (frame == nullptr) {
        return false;
    }
    if (frame[WRENCH_PAYLOAD_DATA_LENGTH] != CR || frame[WRENCH_PAYLOAD_DATA_LENGTH + 1] != LF) {
        return false;
    }
    char payload[WRENCH_PAYLOAD_DATA_LENGTH + 1];
    std::memcpy(payload, frame, WRENCH_PAYLOAD_DATA_LENGTH);
    payload[WRENCH_PAYLOAD_DATA_LENGTH] = '\0';
    return parseWrench(arm, payload, out);
}

bool WrenchController::parseWrench(ArmContext & arm, const char *payload, Wrench6& out) {
    if (payload == nullptr) {
        return false;
    }

    int tick = 0;
    unsigned short raw[6];
    const int n = std::sscanf(payload, "%1d%4hx%4hx%4hx%4hx%4hx%4hx",
        &tick, &raw[0], &raw[1], &raw[2], &raw[3], &raw[4], &raw[5]);
    if (n != 7) {
        return false;
    }

    out.fx = (static_cast<double>(raw[0]) - lbs_val) / arm.calib[0];
    out.fy = (static_cast<double>(raw[1]) - lbs_val) / arm.calib[1];
    out.fz = (static_cast<double>(raw[2]) - lbs_val) / arm.calib[2];
    out.mx = (static_cast<double>(raw[3]) - lbs_val) / arm.calib[3];
    out.my = (static_cast<double>(raw[4]) - lbs_val) / arm.calib[4];
    out.mz = (static_cast<double>(raw[5]) - lbs_val) / arm.calib[5];
    return true;
}

bool WrenchController::buildDefaultNoidArmJointMapping(const ArmContext & arm, const std::vector<std::string> & raw_kdl_joint_names, std::vector<KdlActiveJointMapping> & mapping) {
    mapping.clear();
    mapping.reserve(raw_kdl_joint_names.size());
    std::vector<bool> active_used(arm.active_joint_names.size(), false);

    const std::string elbow = arm.joint_prefix + "_elbow_joint";
    for (const auto & raw_name : raw_kdl_joint_names) {
        std::string active_name = raw_name;
        double multiplier = 1.0;

        if (raw_name == arm.joint_prefix + "_elbow_joint_mimic") {
            active_name = elbow;
            multiplier = -1.0;
        } else if (raw_name == arm.joint_prefix + "_elbow_middle_joint") {
            active_name = elbow;
            multiplier = 0.591222;
        } else if (raw_name == arm.joint_prefix + "_elbow_middle_joint_mimic") {
            active_name = elbow;
            multiplier = 0.408778;
        }

        const int active_index = findActiveJointIndex(arm, active_name);
        if (active_index < 0) {
            RCLCPP_ERROR(this->get_logger(), "KDL joint '%s' maps to active joint '%s', but active joint is not listed. active=[%s]",
                raw_name.c_str(), active_name.c_str(), joinStrings(arm.active_joint_names).c_str());
            return false;
        }

        active_used[static_cast<size_t>(active_index)] = true;
        mapping.push_back({raw_name, active_name, active_index, multiplier, 0.0});
    }

    std::vector<std::string> missing;
    for (size_t i = 0; i < active_used.size(); ++i) {
        if (!active_used[i]) {
            missing.push_back(arm.active_joint_names[i]);
        }
    }

    if (!missing.empty()) {
        RCLCPP_ERROR(this->get_logger(), "KDL chain does not contain active right-arm joints=[%s]. Check base_link='%s' and tip_link='%s'. raw_kdl=[%s]",
            joinStrings(missing).c_str(), arm.base_link.c_str(), arm.tip_link.c_str(), joinStrings(raw_kdl_joint_names).c_str());
        return false;
    }
    return true;
}

bool WrenchController::buildDefaultNoidDynamicsJointMapping(const ArmContext & arm, const std::vector<std::string> & raw_kdl_joint_names, std::vector<KdlActiveJointMapping> & mapping) {
    mapping.clear();
    mapping.reserve(raw_kdl_joint_names.size());
    std::vector<bool> active_used(arm.dynamics_active_joint_names.size(), false);

    const std::string elbow = arm.joint_prefix + "_elbow_joint";
    for (const auto & raw_name : raw_kdl_joint_names) {
        std::string active_name = raw_name;
        double multiplier = 1.0;

        // Preserve exactly the same elbow mimic mapping used by the existing arm-only chain.
        if (raw_name == arm.joint_prefix + "_elbow_joint_mimic") {
            active_name = elbow;
            multiplier = -1.0;
        } else if (raw_name == arm.joint_prefix + "_elbow_middle_joint") {
            active_name = elbow;
            multiplier = 0.591222;
        } else if (raw_name == arm.joint_prefix + "_elbow_middle_joint_mimic") {
            active_name = elbow;
            multiplier = 0.408778;
        }

        const auto it = std::find(arm.dynamics_active_joint_names.begin(), arm.dynamics_active_joint_names.end(), active_name);
        if (it == arm.dynamics_active_joint_names.end()) {
            RCLCPP_ERROR(this->get_logger(), "%s dynamics KDL joint '%s' maps to '%s', but it is not in dynamics joints=[%s].",
                arm.side.c_str(), raw_name.c_str(), active_name.c_str(), joinStrings(arm.dynamics_active_joint_names).c_str());
            return false;
        }

        const int active_index = static_cast<int>(std::distance(arm.dynamics_active_joint_names.begin(), it));
        active_used[static_cast<size_t>(active_index)] = true;
        mapping.push_back({raw_name, active_name, active_index, multiplier, 0.0});
    }

    std::vector<std::string> missing;
    for (size_t i = 0; i < active_used.size(); ++i) {
        if (!active_used[i]) {
            missing.push_back(arm.dynamics_active_joint_names[i]);
        }
    }
    if (!missing.empty()) {
        RCLCPP_ERROR(this->get_logger(), "%s dynamics KDL chain base='%s' tip='%s' misses joints=[%s]. raw_kdl=[%s]",
            arm.side.c_str(), arm.dynamics_base_link.c_str(), arm.tip_link.c_str(), joinStrings(missing).c_str(), joinStrings(raw_kdl_joint_names).c_str());
        return false;
    }
    return true;
}

bool WrenchController::buildKinematicsFromUrdfForArm(ArmContext & arm, const std::string & robot_description) {
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_description, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse robot_description as KDL tree.");
        return false;
    }

    KDL::Chain arm_chain;
    if (!tree.getChain(arm.base_link, arm.tip_link, arm_chain)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get arm KDL chain from base='%s' to tip='%s'.", arm.base_link.c_str(), arm.tip_link.c_str());
        return false;
    }

    std::vector<std::string> raw_arm_joint_names;
    for (unsigned int i = 0; i < arm_chain.getNrOfSegments(); ++i) {
        const auto & joint = arm_chain.getSegment(i).getJoint();
        if (joint.getType() != KDL::Joint::None) {
            raw_arm_joint_names.push_back(joint.getName());
        }
    }

    std::vector<KdlActiveJointMapping> arm_mapping;
    if (!buildDefaultNoidArmJointMapping(arm, raw_arm_joint_names, arm_mapping)) {
        return false;
    }
    if (arm_mapping.size() != raw_arm_joint_names.size() ||
        arm_mapping.size() != arm_chain.getNrOfJoints()) {
        RCLCPP_ERROR(this->get_logger(), "%s: arm KDL joint/mapping size mismatch.", arm.side.c_str());
        return false;
    }

    KDL::Chain dynamics_chain;
    if (!tree.getChain(arm.dynamics_base_link, arm.tip_link, dynamics_chain)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get dynamics KDL chain from base='%s' to tip='%s'.", arm.dynamics_base_link.c_str(), arm.tip_link.c_str());
        return false;
    }

    std::vector<std::string> raw_dynamics_joint_names;
    for (unsigned int i = 0; i < dynamics_chain.getNrOfSegments(); ++i) {
        const auto & joint = dynamics_chain.getSegment(i).getJoint();
        if (joint.getType() != KDL::Joint::None) {
            raw_dynamics_joint_names.push_back(joint.getName());
        }
    }

    std::vector<KdlActiveJointMapping> dynamics_mapping;
    if (!buildDefaultNoidDynamicsJointMapping(
            arm, raw_dynamics_joint_names, dynamics_mapping)) {
        return false;
    }
    if (dynamics_mapping.size() != raw_dynamics_joint_names.size() ||
        dynamics_mapping.size() != dynamics_chain.getNrOfJoints()) {
        RCLCPP_ERROR(this->get_logger(), "%s: dynamics KDL joint/mapping size mismatch.", arm.side.c_str());
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(arm.kdl_mutex);

        arm.chain = arm_chain;
        arm.raw_kdl_joint_names = raw_arm_joint_names;
        arm.kdl_active_mapping = arm_mapping;
        arm.fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(arm.chain);
        arm.jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(arm.chain);

        arm.dynamics_chain = dynamics_chain;
        arm.dynamics_raw_kdl_joint_names = raw_dynamics_joint_names;
        arm.dynamics_kdl_active_mapping = dynamics_mapping;
        arm.dynamics_fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(arm.dynamics_chain);
        arm.dynamics_jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(arm.dynamics_chain);
    }
    arm.kinematics_ready.store(true);
    arm.dynamics_state_reset_requested.store(true);

    std::vector<std::string> arm_map_strings;
    for (const auto & m : arm_mapping) {
        std::ostringstream ss;
        ss << m.kdl_joint_name << "->" << m.active_joint_name << "*" << m.multiplier;
        if (std::abs(m.offset) > 1e-12) {
            ss << "+" << m.offset;
        }
        arm_map_strings.push_back(ss.str());
    }

    std::vector<std::string> dynamics_map_strings;
    for (const auto & m : dynamics_mapping) {
        std::ostringstream ss;
        ss << m.kdl_joint_name << "->" << m.active_joint_name << "*" << m.multiplier;
        if (std::abs(m.offset) > 1e-12) {
            ss << "+" << m.offset;
        }
        dynamics_map_strings.push_back(ss.str());
    }

    RCLCPP_INFO(this->get_logger(), "%s arm reflection chain ready: base=%s tip=%s mapping=[%s]",
        arm.side.c_str(), arm.base_link.c_str(), arm.tip_link.c_str(), joinStrings(arm_map_strings).c_str());
    RCLCPP_INFO(this->get_logger(), "%s payload dynamics chain ready: base=%s tip=%s mapping=[%s]",
        arm.side.c_str(), arm.dynamics_base_link.c_str(), arm.tip_link.c_str(), joinStrings(dynamics_map_strings).c_str());
    return true;
}

bool WrenchController::getActiveJointPositions(const ArmContext & arm, std::vector<double> & active_q) {
    if (!arm.kinematics_ready.load() || !have_joint_state.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(joint_mutex);
    if ((now() - latest_joint_stamp).seconds() > 0.1) {
        return false;
    }

    active_q.assign(arm.active_joint_names.size(), 0.0);
    for (size_t i = 0; i < arm.active_joint_names.size(); ++i) {
        const auto it = latest_joint_position.find(arm.active_joint_names[i]);
        if (it == latest_joint_position.end()) {
            return false;
        }
        active_q[i] = it->second;
    }
    return true;
}

bool WrenchController::getActiveJointState(const ArmContext & arm, std::vector<double> & active_q, std::vector<double> & active_qdot, rclcpp::Time & stamp, bool & velocity_available) {
    if (!arm.kinematics_ready.load() || !have_joint_state.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(joint_mutex);
    if ((now() - latest_joint_stamp).seconds() > 0.1) {
        return false;
    }

    active_q.assign(arm.active_joint_names.size(), 0.0);
    active_qdot.assign(arm.active_joint_names.size(), 0.0);
    velocity_available = true;

    for (size_t i = 0; i < arm.active_joint_names.size(); ++i) {
        const auto pos_it = latest_joint_position.find(arm.active_joint_names[i]);
        if (pos_it == latest_joint_position.end()) {
            return false;
        }
        active_q[i] = pos_it->second;

        const auto vel_it = latest_joint_velocity.find(arm.active_joint_names[i]);
        if (vel_it == latest_joint_velocity.end() || !std::isfinite(vel_it->second)) {
            velocity_available = false;
        } else {
            active_qdot[i] = vel_it->second;
        }
    }
    stamp = latest_joint_stamp;
    return true;
}


bool WrenchController::getDynamicsJointState(const ArmContext & arm, std::vector<double> & dynamics_q, std::vector<double> & dynamics_qdot, rclcpp::Time & stamp, bool & velocity_available) {
    if (!arm.kinematics_ready.load() || !have_joint_state.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(joint_mutex);
    if ((now() - latest_joint_stamp).seconds() > 0.1) {
        return false;
    }

    dynamics_q.assign(arm.dynamics_active_joint_names.size(), 0.0);
    dynamics_qdot.assign(arm.dynamics_active_joint_names.size(), 0.0);
    velocity_available = true;

    for (size_t i = 0; i < arm.dynamics_active_joint_names.size(); ++i) {
        const std::string & name = arm.dynamics_active_joint_names[i];
        const auto pos_it = latest_joint_position.find(name);
        if (pos_it == latest_joint_position.end()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s payload dynamics waiting for /joint_states position '%s'.", arm.side.c_str(), name.c_str());
            return false;
        }
        dynamics_q[i] = pos_it->second;

        const auto vel_it = latest_joint_velocity.find(name);
        if (vel_it == latest_joint_velocity.end() || !std::isfinite(vel_it->second)) {
            velocity_available = false;
        } else {
            dynamics_qdot[i] = vel_it->second;
        }
    }

    stamp = latest_joint_stamp;
    return true;
}

int WrenchController::findActiveJointIndex(const ArmContext & arm, const std::string & name) {
    const auto it = std::find(arm.active_joint_names.begin(), arm.active_joint_names.end(), name);
    if (it == arm.active_joint_names.end()) {
        return -1;
    }
    return static_cast<int>(std::distance(arm.active_joint_names.begin(), it));
}

KDL::Frame WrenchController::makeSensorFrameFromParams(const ArmContext & arm) const {
    if (arm.sensor_xyz_in_tip.size() != 3 || arm.sensor_rpy_in_tip.size() != 3) {
        return KDL::Frame::Identity();
    }
    return KDL::Frame(
        KDL::Rotation::RPY(arm.sensor_rpy_in_tip[0], arm.sensor_rpy_in_tip[1], arm.sensor_rpy_in_tip[2]),
        KDL::Vector(arm.sensor_xyz_in_tip[0], arm.sensor_xyz_in_tip[1], arm.sensor_xyz_in_tip[2])
    );
}

KDL::Vector WrenchController::cross(const KDL::Vector & a, const KDL::Vector & b) {
    return KDL::Vector(
      a.y() * b.z() - a.z() * b.y(),
      a.z() * b.x() - a.x() * b.z(),
      a.x() * b.y() - a.y() * b.x());
}

WrenchController::Wrench6 WrenchController::subtractWrench(const Wrench6 & lhs, const Wrench6 & rhs) {
    return {
        lhs.fx - rhs.fx, lhs.fy - rhs.fy, lhs.fz - rhs.fz,
        lhs.mx - rhs.mx, lhs.my - rhs.my, lhs.mz - rhs.mz
    };
}

WrenchController::Wrench6 WrenchController::addWrench(const Wrench6 & lhs, const Wrench6 & rhs) {
    return {
        lhs.fx + rhs.fx, lhs.fy + rhs.fy, lhs.fz + rhs.fz,
        lhs.mx + rhs.mx, lhs.my + rhs.my, lhs.mz + rhs.mz
    };
}

WrenchController::Wrench6 WrenchController::scaleWrench(const Wrench6 & wrench, double scale) {
    return {
        wrench.fx * scale, wrench.fy * scale, wrench.fz * scale,
        wrench.mx * scale, wrench.my * scale, wrench.mz * scale
    };
}

KDL::Vector WrenchController::clampVectorNorm(const KDL::Vector & value, double max_norm) {
    if (max_norm <= 0.0) {
        return value;
    }
    const double n = value.Norm();
    if (!std::isfinite(n) || n <= max_norm || n <= 1e-12) {
        return value;
    }
    return value * (max_norm / n);
}

bool WrenchController::computePayloadModelWrenchInSensor(ArmContext & arm, Wrench6 & payload_wrench) {
    if (arm.payload_mass_kg <= 0.0) {
        payload_wrench = Wrench6{};
        return true;
    }

    std::vector<double> dynamics_q;
    std::vector<double> dynamics_qdot;
    rclcpp::Time joint_stamp;
    bool velocity_available = false;
    if (!getDynamicsJointState(arm, dynamics_q, dynamics_qdot, joint_stamp, velocity_available)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(arm.kdl_mutex);
    if (!arm.dynamics_fk_solver || arm.dynamics_kdl_active_mapping.empty()) {
        return false;
    }

    KDL::JntArray q_raw(arm.dynamics_kdl_active_mapping.size());
    KDL::JntArray qdot_raw(arm.dynamics_kdl_active_mapping.size());
    for (size_t i = 0; i < arm.dynamics_kdl_active_mapping.size(); ++i) {
        const auto & map = arm.dynamics_kdl_active_mapping[i];
        if (map.active_index < 0 ||
            static_cast<size_t>(map.active_index) >= dynamics_q.size()) {
            return false;
        }
        q_raw(i) = map.multiplier * dynamics_q[static_cast<size_t>(map.active_index)] + map.offset;
        qdot_raw(i) = velocity_available ? map.multiplier * dynamics_qdot[static_cast<size_t>(map.active_index)] : 0.0;
    }

    KDL::Frame dynamics_base_T_tip;
    if (arm.dynamics_fk_solver->JntToCart(q_raw, dynamics_base_T_tip) < 0) {
        return false;
    }

    KDL::Jacobian jac(q_raw.rows());
    if (velocity_available && (!arm.dynamics_jac_solver || arm.dynamics_jac_solver->JntToJac(q_raw, jac) < 0)) {
        velocity_available = false;
    }

    const KDL::Rotation dynamics_base_R_sensor = dynamics_base_T_tip.M * arm.tip_T_sensor.M;

    const KDL::Vector gravity_dynamics_base(arm.gravity_vector_in_base[0], arm.gravity_vector_in_base[1], arm.gravity_vector_in_base[2]);
    const KDL::Vector payload_force_dynamics_base = gravity_dynamics_base * arm.payload_mass_kg;
    const KDL::Vector payload_force_sensor = dynamics_base_R_sensor.Inverse() * payload_force_dynamics_base;

    const KDL::Vector com_sensor(arm.payload_com_in_sensor[0], arm.payload_com_in_sensor[1], arm.payload_com_in_sensor[2]);
    const KDL::Vector payload_moment_sensor =cross(com_sensor, payload_force_sensor);

    Wrench6 gravity_wrench = {
        payload_force_sensor.x(), payload_force_sensor.y(), payload_force_sensor.z(),
        payload_moment_sensor.x(), payload_moment_sensor.y(), payload_moment_sensor.z()
    };

    const KDL::Vector tip_to_com_tip = arm.tip_T_sensor.p + arm.tip_T_sensor.M * com_sensor;
    const KDL::Vector tip_to_com_dynamics_base = dynamics_base_T_tip.M * tip_to_com_tip;
    const KDL::Vector com_position_dynamics_base = dynamics_base_T_tip.p + tip_to_com_dynamics_base;

    KDL::Vector com_velocity_dynamics_base(0.0, 0.0, 0.0);
    KDL::Vector angular_velocity_dynamics_base(0.0, 0.0, 0.0);

    if (velocity_available) {
        KDL::Vector tip_velocity_dynamics_base(0.0, 0.0, 0.0);
        for (unsigned int j = 0; j < jac.columns(); ++j) {
            const double qd = qdot_raw(j);
            tip_velocity_dynamics_base = tip_velocity_dynamics_base + KDL::Vector(jac(0, j), jac(1, j), jac(2, j)) * qd;
            angular_velocity_dynamics_base = angular_velocity_dynamics_base + KDL::Vector(jac(3, j), jac(4, j), jac(5, j)) * qd;
        }
        com_velocity_dynamics_base = tip_velocity_dynamics_base + cross(angular_velocity_dynamics_base, tip_to_com_dynamics_base);
    }

    KDL::Vector linear_acceleration_dynamics_base(0.0, 0.0, 0.0);
    KDL::Vector angular_acceleration_dynamics_base(0.0, 0.0, 0.0);
    const bool reset_motion = arm.dynamics_state_reset_requested.exchange(false);

    if (reset_motion) {
        arm.payload_model_filter_initialized = false;
    }

    if (!arm.motion_state_initialized || reset_motion) {
        arm.motion_state_initialized = true;
        arm.motion_prev_stamp = joint_stamp;
        arm.prev_com_position_base = com_position_dynamics_base;
        arm.prev_com_velocity_base = com_velocity_dynamics_base;
        arm.prev_angular_velocity_base = angular_velocity_dynamics_base;
    } else {
        const double dt = (joint_stamp - arm.motion_prev_stamp).seconds();
        if (std::isfinite(dt) &&
            dt >= arm.dynamics_min_dt &&
            dt <= arm.dynamics_max_dt) {

            if (velocity_available) {
                linear_acceleration_dynamics_base = (com_velocity_dynamics_base - arm.prev_com_velocity_base) * (1.0 / dt);
                angular_acceleration_dynamics_base = (angular_velocity_dynamics_base - arm.prev_angular_velocity_base) * (1.0 / dt);
            } else {
                const KDL::Vector estimated_velocity = (com_position_dynamics_base - arm.prev_com_position_base) * (1.0 / dt);
                linear_acceleration_dynamics_base = (estimated_velocity - arm.prev_com_velocity_base) * (1.0 / dt);
                com_velocity_dynamics_base = estimated_velocity;
            }

            linear_acceleration_dynamics_base = clampVectorNorm(linear_acceleration_dynamics_base, arm.max_linear_acceleration);
            angular_acceleration_dynamics_base = clampVectorNorm(angular_acceleration_dynamics_base, arm.max_angular_acceleration);

            arm.motion_prev_stamp = joint_stamp;
            arm.prev_com_position_base = com_position_dynamics_base;
            arm.prev_com_velocity_base = com_velocity_dynamics_base;
            arm.prev_angular_velocity_base = angular_velocity_dynamics_base;
        } else if (!std::isfinite(dt) || dt > arm.dynamics_max_dt || dt < 0.0) {
            arm.motion_prev_stamp = joint_stamp;
            arm.prev_com_position_base = com_position_dynamics_base;
            arm.prev_com_velocity_base = com_velocity_dynamics_base;
            arm.prev_angular_velocity_base = angular_velocity_dynamics_base;
        }
    }

    const KDL::Vector inertia_force_dynamics_base = linear_acceleration_dynamics_base * (-arm.payload_mass_kg);
    const KDL::Vector inertia_force_sensor = dynamics_base_R_sensor.Inverse() * inertia_force_dynamics_base;

    KDL::Vector inertia_moment_sensor = cross(com_sensor, inertia_force_sensor);

    if (arm.payload_inertia_diag_in_sensor.size() == 3) {
        const KDL::Vector omega_sensor = dynamics_base_R_sensor.Inverse() * angular_velocity_dynamics_base;
        const KDL::Vector alpha_sensor = dynamics_base_R_sensor.Inverse() * angular_acceleration_dynamics_base;

        const KDL::Vector i_omega(arm.payload_inertia_diag_in_sensor[0] * omega_sensor.x(), arm.payload_inertia_diag_in_sensor[1] * omega_sensor.y(), arm.payload_inertia_diag_in_sensor[2] * omega_sensor.z());
        const KDL::Vector i_alpha(arm.payload_inertia_diag_in_sensor[0] * alpha_sensor.x(), arm.payload_inertia_diag_in_sensor[1] * alpha_sensor.y(), arm.payload_inertia_diag_in_sensor[2] * alpha_sensor.z());

        inertia_moment_sensor = inertia_moment_sensor - i_alpha - cross(omega_sensor, i_omega);
    }

    const Wrench6 inertia_wrench = {
        inertia_force_sensor.x(), inertia_force_sensor.y(), inertia_force_sensor.z(),
        inertia_moment_sensor.x(), inertia_moment_sensor.y(), inertia_moment_sensor.z()
    };

    payload_wrench = addWrench(gravity_wrench, inertia_wrench);
    return true;
}

bool WrenchController::compensatePayloadDynamics(
    ArmContext & arm, const Wrench6 & measured, Wrench6 & compensated) {
    if (arm.payload_mass_kg <= 0.0) {
        compensated = measured;
        return true;
    }

    Wrench6 current_payload_model;
    if (!computePayloadModelWrenchInSensor(arm, current_payload_model)) {
        return false;
    }

    if (!arm.payload_model_filter_initialized) {
        arm.filtered_payload_model = current_payload_model;
        arm.payload_model_filter_initialized = true;
    } else {
        const double a = std::clamp(arm.lowpass_alpha, 0.0, 1.0);
        arm.filtered_payload_model.fx = a * current_payload_model.fx + (1.0 - a) * arm.filtered_payload_model.fx;
        arm.filtered_payload_model.fy = a * current_payload_model.fy + (1.0 - a) * arm.filtered_payload_model.fy;
        arm.filtered_payload_model.fz = a * current_payload_model.fz + (1.0 - a) * arm.filtered_payload_model.fz;
        arm.filtered_payload_model.mx = a * current_payload_model.mx + (1.0 - a) * arm.filtered_payload_model.mx;
        arm.filtered_payload_model.my = a * current_payload_model.my + (1.0 - a) * arm.filtered_payload_model.my;
        arm.filtered_payload_model.mz = a * current_payload_model.mz + (1.0 - a) * arm.filtered_payload_model.mz;
    }

    if (arm.payload_reference_pending.exchange(false) || !arm.payload_reference_valid.load()) {
        arm.payload_reference_wrench = arm.filtered_payload_model;
        arm.payload_reference_valid.store(true);
        compensated = measured;
        RCLCPP_INFO(this->get_logger(), "%s payload dynamics reference captured: F=[%.3f %.3f %.3f] M=[%.3f %.3f %.3f]",
            arm.side.c_str(), arm.filtered_payload_model.fx, arm.filtered_payload_model.fy, arm.filtered_payload_model.fz,
            arm.filtered_payload_model.mx, arm.filtered_payload_model.my, arm.filtered_payload_model.mz);
        return true;
    }

    const Wrench6 payload_change = subtractWrench(arm.filtered_payload_model, arm.payload_reference_wrench);
    compensated = subtractWrench(measured, scaleWrench(payload_change, arm.gravity_compensation_sign));
    return true;
}

bool WrenchController::computeTauFromWrench(ArmContext & arm, const Wrench6 & sensor_wrench, std::vector<double> & tau_out) {
    std::vector<double> active_q;
    if (!getActiveJointPositions(arm, active_q)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(arm.kdl_mutex);
    if (!arm.fk_solver || !arm.jac_solver || arm.kdl_active_mapping.empty()) {
        return false;
    }

    KDL::JntArray q_raw(arm.kdl_active_mapping.size());
    for (size_t i = 0; i < arm.kdl_active_mapping.size(); ++i) {
        const auto & map = arm.kdl_active_mapping[i];
        if (map.active_index < 0 || static_cast<size_t>(map.active_index) >= active_q.size()) {
            return false;
        }
        q_raw(i) = map.multiplier * active_q[static_cast<size_t>(map.active_index)] +
        map.offset;
    }

    KDL::Frame base_T_tip;
    if (arm.fk_solver->JntToCart(q_raw, base_T_tip) < 0) {
        return false;
    }

    KDL::Jacobian jac(q_raw.rows());
    if (arm.jac_solver->JntToJac(q_raw, jac) < 0) {
        return false;
    }

    KDL::Vector f_sensor(sensor_wrench.fx, sensor_wrench.fy, sensor_wrench.fz);
    KDL::Vector m_sensor(sensor_wrench.mx, sensor_wrench.my, sensor_wrench.mz);

    KDL::Vector f_tip = arm.tip_T_sensor.M * f_sensor;
    KDL::Vector m_tip = arm.tip_T_sensor.M * m_sensor + cross(arm.tip_T_sensor.p, f_tip);
    KDL::Vector f_base = base_T_tip.M * f_tip;
    KDL::Vector m_base = base_T_tip.M * m_tip;

    std::vector<double> tau_raw(jac.columns(), 0.0);
    for (unsigned int j = 0; j < jac.columns(); ++j) {
        tau_raw[j] =
        jac(0, j) * f_base.x() + jac(1, j) * f_base.y() + jac(2, j) * f_base.z() +
        jac(3, j) * m_base.x() + jac(4, j) * m_base.y() + jac(5, j) * m_base.z();
    }

    if (tau_raw.size() != arm.kdl_active_mapping.size()) {
        return false;
    }

    tau_out.assign(arm.active_joint_names.size(), 0.0);
    for (size_t i = 0; i < tau_raw.size(); ++i) {
        const auto & map = arm.kdl_active_mapping[i];
        if (map.active_index < 0 || static_cast<size_t>(map.active_index) >= tau_out.size()) {
            continue;
        }
        tau_out[static_cast<size_t>(map.active_index)] += map.multiplier * tau_raw[i];
    }
    return true;
}

aero_controller_msgs::msg::Current WrenchController::makeOutputFromBaseOrNeutral() {
    aero_controller_msgs::msg::Current out;
    fillNeutral(out);

    std::lock_guard<std::mutex> lock(current_raw_mutex);
    if (!have_current_raw.load()) {
        return out;
    }
    if ((now() - latest_current_raw_stamp).seconds() > 0.1) {
        return out;
    }

    // Only the hand current value is used as raw data.
    out.data[right_hand_aero_id * 2] = latest_current_raw.data[right_hand_aero_id * 2];
    out.data[right_hand_aero_id * 2 + 1] = latest_current_raw.data[right_hand_aero_id * 2 + 1];
    out.data[left_hand_aero_id * 2] = latest_current_raw.data[left_hand_aero_id * 2];
    out.data[left_hand_aero_id * 2 + 1] = latest_current_raw.data[left_hand_aero_id * 2 + 1];

    const size_t n_pos = std::min(out.pos_data.size(), latest_current_raw.pos_data.size());
    std::copy_n(latest_current_raw.pos_data.begin(), n_pos, out.pos_data.begin());

    return out;
}

bool WrenchController::isProtectedTracerIndex(int tracer_index) const {
    for (const auto id : protected_arm_joint_indices) {
        if (tracer_index == static_cast<int>(id)) {
            return true;
        }
    }
    return false;
}

void WrenchController::wrenchLoop() {
    using clock = std::chrono::steady_clock;
    const double safe_rate = std::max(1.0, sample_rate_hz);
    const auto period = std::chrono::duration<double>(1.0 / safe_rate);
    auto next = clock::now();

    while (rclcpp::ok() && running_.load()) {
        next += std::chrono::duration_cast<clock::duration>(period);

        std::vector<double> right_tau;
        std::vector<double> left_tau;
        const bool right_reflection = processArmSample(right_arm_, right_tau);
        const bool left_reflection = processArmSample(left_arm_, left_tau);

        publishMergedCurrent(
            right_reflection ? &right_tau : nullptr, right_reflection,
            left_reflection ? &left_tau : nullptr, left_reflection
        );

        const auto now_tp = clock::now();
        if (now_tp > next + std::chrono::duration_cast<clock::duration>(period)) {
        next = now_tp;
        }
        std::this_thread::sleep_until(next);
    }
}

bool WrenchController::processArmSample(ArmContext & arm, std::vector<double> & tau_out) {
    if (!arm.sensor_ready.load()) {
        return false;
    }

    Wrench6 raw;
    if (!readSample(arm, raw)) {
        return false;
    }

    if (!arm.filter_initialized.exchange(true)) {
        arm.filtered = raw;
    } else {
        const double a = std::clamp(arm.lowpass_alpha, 0.0, 1.0);
        arm.filtered.fx = a * raw.fx + (1.0 - a) * arm.filtered.fx;
        arm.filtered.fy = a * raw.fy + (1.0 - a) * arm.filtered.fy;
        arm.filtered.fz = a * raw.fz + (1.0 - a) * arm.filtered.fz;
        arm.filtered.mx = a * raw.mx + (1.0 - a) * arm.filtered.mx;
        arm.filtered.my = a * raw.my + (1.0 - a) * arm.filtered.my;
        arm.filtered.mz = a * raw.mz + (1.0 - a) * arm.filtered.mz;
    }

    Wrench6 compensated;
    if (!compensatePayloadDynamics(arm, arm.filtered, compensated)) {
        return false;
    }

    if (publish_wrench_debug && arm.wrench_pub) {
        publishWrenchDebug(arm, arm.filtered, arm.sensor_frame_id);
    }

    if (publish_wrench_debug && arm.dynamics_compensated_wrench_pub) {
        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = arm.sensor_frame_id;
        msg.wrench.force.x = compensated.fx;
        msg.wrench.force.y = compensated.fy;
        msg.wrench.force.z = compensated.fz;
        msg.wrench.torque.x = compensated.mx;
        msg.wrench.torque.y = compensated.my;
        msg.wrench.torque.z = compensated.mz;
        arm.dynamics_compensated_wrench_pub->publish(msg);
    }

    Wrench6 gated = compensated;
    gated.fx = applyDeadband(gated.fx, arm.force_deadband);
    gated.fy = applyDeadband(gated.fy, arm.force_deadband);
    gated.fz = applyDeadband(gated.fz, arm.force_deadband);
    gated.mx = applyDeadband(gated.mx, arm.torque_deadband);
    gated.my = applyDeadband(gated.my, arm.torque_deadband);
    gated.mz = applyDeadband(gated.mz, arm.torque_deadband);

    if (arm.reflection_state_reset_requested.exchange(false)) {
        resetReflectionState(arm);
    }

    if (!on_force_feedback_.load()) {
        resetReflectionState(arm);
        return false;
    }

    const double force_norm = norm3(gated.fx, gated.fy, gated.fz);

    const bool above_on_threshold = force_norm >= arm.force_on_threshold;
    const bool below_off_threshold = force_norm <= arm.force_off_threshold;

    if (!arm.reflection_active) {
        arm.reflection_off_count = 0;
        if (above_on_threshold) {
            ++arm.reflection_on_count;
        } else {
            arm.reflection_on_count = 0;
        }
        if (arm.reflection_on_count >= std::max(1, arm.reflection_on_samples)) {
            arm.reflection_active = true;
            arm.reflection_on_count = 0;
        }
    } else {
        arm.reflection_on_count = 0;
        if (below_off_threshold) {
            ++arm.reflection_off_count;
        } else {
            arm.reflection_off_count = 0;
        }
        if (arm.reflection_off_count >= std::max(1, arm.reflection_off_samples)) {
            arm.reflection_active = false;
            arm.reflection_off_count = 0;
        }
    }

    if (!arm.reflection_active) {
        return false;
    }

    if (!computeTauFromWrench(arm, gated, tau_out)) {
        return false;
    }

    if (publish_tau_debug && arm.tau_pub) {
        publishTauDebug(arm, tau_out, arm.active_joint_names);
    }
    return true;
}

void WrenchController::resetReflectionState(ArmContext & arm) {
    arm.reflection_active = false;
    arm.reflection_on_count = 0;
    arm.reflection_off_count = 0;
}

void WrenchController::resetReflectionDelta(ArmContext & arm) {
    std::fill(arm.last_current_delta.begin(), arm.last_current_delta.end(), 0.0);
    arm.last_current_publish_time = now();
}

void WrenchController::resetAllReflectionDeltas() {
    resetReflectionDelta(right_arm_);
    resetReflectionDelta(left_arm_);
}

void WrenchController::publishPassThroughOrNeutral() {
    resetAllReflectionDeltas();
    current_pub_->publish(makeOutputFromBaseOrNeutral());
}

void WrenchController::publishNeutralCurrent() {
    resetAllReflectionDeltas();
    aero_controller_msgs::msg::Current msg;
    fillNeutral(msg);

    std::lock_guard<std::mutex> lock(current_raw_mutex);
    if (have_current_raw.load()) {
        const size_t n_pos = std::min(msg.pos_data.size(), latest_current_raw.pos_data.size());
        std::copy_n(latest_current_raw.pos_data.begin(), n_pos, msg.pos_data.begin());
    }
    current_pub_->publish(msg);
}

void WrenchController::publishMergedCurrent(const std::vector<double> * right_tau, bool right_has_reflection, const std::vector<double> * left_tau, bool left_has_reflection) {
    auto msg = makeOutputFromBaseOrNeutral();

    const bool global_enabled = on_force_feedback_.load();

    if (!global_enabled) {
        resetReflectionState(right_arm_);
        resetReflectionState(left_arm_);
        resetAllReflectionDeltas();
        current_pub_->publish(msg);
        return;
    }

    if (right_has_reflection && right_tau != nullptr) {
        mergeArmCurrent(msg, right_arm_, *right_tau);
    } else {
        releaseArmCurrent(msg, right_arm_);
    }

    if (left_has_reflection && left_tau != nullptr) {
        mergeArmCurrent(msg, left_arm_, *left_tau);
    } else {
        releaseArmCurrent(msg, left_arm_);
    }
    current_pub_->publish(msg);
}

void WrenchController::mergeArmCurrent(aero_controller_msgs::msg::Current & msg, ArmContext & arm, const std::vector<double> & tau) {
    const rclcpp::Time t = now();
    double dt = (t - arm.last_current_publish_time).seconds();
    if (!(dt > 0.0) || dt > 1.0) {
        dt = 1.0 / std::max(1.0, sample_rate_hz);
    }
    arm.last_current_publish_time = t;

    if (tau.size() != arm.arm_joint_indices.size() || tau.size() != arm.current_signs.size() || tau.size() != arm.current_gains.size() || tau.size() != arm.last_current_delta.size()) {
        resetReflectionDelta(arm);
        return;
    }

    const double attack_step = std::max(0.0, arm.current_attack_rate_per_sec) * dt;
    const double release_step = std::max(0.0, arm.current_release_rate_per_sec) * dt;

    for (size_t axis = 0; axis < tau.size(); ++axis) {
        const int tracer_index = static_cast<int>(arm.arm_joint_indices[axis]);
        if (tracer_index < 0 || tracer_index >= MOTOR_COUNT || isProtectedTracerIndex(tracer_index)) {
            continue;
        }

        const double tau_i = applyDeadband(tau[axis], arm.tau_deadband);

        double target_delta = std::abs(arm.current_signs[axis] * arm.current_gains[axis] * tau_i);
        target_delta = std::clamp(target_delta, 0.0, static_cast<double>(arm.max_current_delta));

        const double previous = std::max(0.0, arm.last_current_delta[axis]);
        double limited_delta = target_delta;

        if (target_delta > previous && attack_step > 0.0) {
            limited_delta = std::min(target_delta, previous + attack_step);
        } else if (target_delta < previous && release_step > 0.0) {
            limited_delta = std::max(target_delta, previous - release_step);
        }
        arm.last_current_delta[axis] = limited_delta;

        const int current_count = static_cast<int>(std::llround(limited_delta));
        const int protocol_current = convertCurrentValue(current_count);
        setCurrentWord(msg, tracer_index, clampCurrent(protocol_current));
    }
}

void WrenchController::releaseArmCurrent( aero_controller_msgs::msg::Current & msg, ArmContext & arm) {
    const rclcpp::Time t = now();
    double dt = (t - arm.last_current_publish_time).seconds();
    if (!(dt > 0.0) || dt > 1.0) {
        dt = 1.0 / std::max(1.0, sample_rate_hz);
    }
    arm.last_current_publish_time = t;

    const double release_step = std::max(0.0, arm.current_release_rate_per_sec) * dt;

    for (size_t axis = 0; axis < arm.arm_joint_indices.size(); ++axis) {
        const int tracer_index = static_cast<int>(arm.arm_joint_indices[axis]);
        if (tracer_index < 0 || tracer_index >= MOTOR_COUNT ||
            isProtectedTracerIndex(tracer_index)) {
            continue;
        }

        const double previous = axis < arm.last_current_delta.size() ? std::max(0.0, arm.last_current_delta[axis]) : 0.0;
        const double limited_delta = release_step > 0.0 ? std::max(0.0, previous - release_step) : 0.0;

        if (axis < arm.last_current_delta.size()) {
            arm.last_current_delta[axis] = limited_delta;
        }

        const int current_count = static_cast<int>(std::llround(limited_delta));
        const int protocol_current = convertCurrentValue(current_count);
        setCurrentWord(msg, tracer_index, clampCurrent(protocol_current));
    }
}

void WrenchController::publishWrenchDebug(ArmContext & arm, const Wrench6 & wrench, const std::string & frame_id) {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id;
    msg.wrench.force.x = wrench.fx;
    msg.wrench.force.y = wrench.fy;
    msg.wrench.force.z = wrench.fz;
    msg.wrench.torque.x = wrench.mx;
    msg.wrench.torque.y = wrench.my;
    msg.wrench.torque.z = wrench.mz;
    arm.wrench_pub->publish(msg);
}

void WrenchController::publishTauDebug(ArmContext & arm, const std::vector<double> & tau, const std::vector<std::string> & names) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = names;
    msg.effort = tau;
    arm.tau_pub->publish(msg);
}

void WrenchController::publishToFLoop() {
    std::vector<uint8_t> current_copy;
    uint16_t r_tof_range = 0x1FFE;
    uint16_t l_tof_range = 0x1FFE;
    {
        std::lock_guard<std::mutex> lock(current_raw_mutex);
        r_tof_range = static_cast<uint16_t>(latest_current_raw.data[right_tof_id * 2]) << 8 |
                        static_cast<uint16_t>(latest_current_raw.data[right_tof_id * 2 + 1]);
        l_tof_range = static_cast<uint16_t>(latest_current_raw.data[left_tof_id * 2]) << 8 |
                        static_cast<uint16_t>(latest_current_raw.data[left_tof_id * 2 + 1]);
    }

    sensor_msgs::msg::LaserScan msg;
    msg.ranges.resize(2);
    msg.header.stamp = now();
    msg.range_min = tof_range_min;
    msg.range_max = tof_range_max;
    msg.ranges[0] = static_cast<float>(r_tof_range);
    msg.ranges[1] = static_cast<float>(l_tof_range);
    tof_raw_pub_->publish(msg);
}

void WrenchController::fillNeutral(aero_controller_msgs::msg::Current & msg) const {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        setCurrentWord(msg, i, neutral_current);
    }
}

// the current value conversion function for dedicated protocol
int WrenchController::convertCurrentValue(int current_val) {
    // divide val by 20 to limit it to -127 to 127.
    int driver_current_val = current_val / 20;
    driver_current_val = std::clamp(driver_current_val, -127, 127);
    // stored as a signed 8-bit value.
    const int8_t driver_current_val_hex = static_cast<int8_t>(driver_current_val);
    // interpret the same bit sequence as an unsigned 8-bit value.
    const uint8_t driver_current_val_hex_u = static_cast<uint8_t>(driver_current_val_hex);
    // Multiply the converted value by 50.
    const int res = static_cast<int>(driver_current_val_hex_u) * 50;
    return res;
}

void WrenchController::setCurrentWord(aero_controller_msgs::msg::Current & msg, int current_index, uint16_t raw) const {
    const int idx = current_index * 2;
    if (idx < 0 || idx + 1 >= static_cast<int>(msg.data.size())) {
      return;
    }
    msg.data[idx] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    msg.data[idx + 1] = static_cast<uint8_t>(raw & 0xFF);
}

uint16_t WrenchController::clampCurrent(int value) const {
    return static_cast<uint16_t>(std::clamp(value, 0, 0xFFFF));
}

double WrenchController::applyDeadband(double value, double deadband) {
    if (std::abs(value) <= deadband) {
        return 0.0;
    }
    return value > 0.0 ? value - deadband : value + deadband;
}

double WrenchController::norm3(double x, double y, double z) {
    return std::sqrt(x * x + y * y + z * z);
}

std::string WrenchController::joinStrings(const std::vector<std::string> & values) {
    std::ostringstream oss;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
        oss << ", ";
        }
        oss << values[i];
    }
    return oss.str();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WrenchController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
