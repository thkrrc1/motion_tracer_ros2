#include "motion_tracer_ros2/wrench_controller.hpp"

WrenchController::WrenchController() : 
    Node("wrench_controller_node") {

    configureArm(
        right_arm_, "right", "r",
        "/dev/force_sensor_right", "r_arm_link", "r_hand_link",
        {0.0, 0.0, -0.015}, {M_PI, 0.0, -M_PI / 2.0},
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
        {0, 1, 2, 3, 4, 5, 6}
    );

    configureArm(
        left_arm_, "left", "l",
        "/dev/force_sensor_left", "l_arm_link", "l_hand_link",
        {0.0, 0.0, -0.015}, {M_PI, 0.0, -M_PI / 2.0},
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
        {15, 16, 17, 18, 19, 20, 21}
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
    }
    if (publish_tau_debug) {
        right_arm_.tau_pub = create_publisher<sensor_msgs::msg::JointState>("/force_sensor/right/reflection_tau", sensor_qos);
        left_arm_.tau_pub = create_publisher<sensor_msgs::msg::JointState>("/force_sensor/left/reflection_tau", sensor_qos);
    }

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
    const std::vector<int64_t> & default_arm_indices) {

    arm.side = side;
    arm.joint_prefix = joint_prefix;
    arm.device = default_device;
    arm.sensor_frame_id = joint_prefix + "_force_sensor";
    arm.base_link = default_base_link;
    arm.tip_link = default_tip_link;
    arm.sensor_xyz_in_tip = default_sensor_xyz_in_tip;
    arm.sensor_rpy_in_tip = default_sensor_rpy_in_tip;
    arm.active_joint_names = default_active_joint_names;
    arm.arm_joint_indices = default_arm_indices;
    arm.current_signs = default_current_signs;
    arm.current_gains = default_current_gains;
    arm.last_current_delta.assign(arm.active_joint_names.size(), 0.0);
    arm.tip_T_sensor = makeSensorFrameFromParams(arm);
}

bool WrenchController::validateArmConfiguration(const ArmContext & arm) const {

    if (arm.sensor_xyz_in_tip.size() != 3 || arm.sensor_rpy_in_tip.size() != 3) {
        RCLCPP_ERROR(this->get_logger(), "%s sensor xyz/rpy parameters must each have three values.", arm.side.c_str());
        return false;
    }

    const size_t n = arm.active_joint_names.size();
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
    latest_joint_stamp = msg->header.stamp;
    if (latest_joint_stamp.nanoseconds() == 0) {
        latest_joint_stamp = now();
    }
    have_joint_state.store(true);
}

void WrenchController::onForceFeedbackCallback(const std_msgs::msg::Bool & msg) {
    on_force_feedback_.store(msg.data);
}

void WrenchController::tareCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    right_arm_.tare_requested.store(true);
    left_arm_.tare_requested.store(true);
    right_arm_.filter_initialized.store(false);
    left_arm_.filter_initialized.store(false);
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
        if (arm.continuous_rx_buffer[i] != CR && arm.continuous_rx_buffer[i + 1] != LF) {
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

bool WrenchController::buildKinematicsFromUrdfForArm(ArmContext & arm, const std::string & robot_description) {
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_description, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse robot_description as KDL tree.");
        return false;
    }

    KDL::Chain chain;
    if (!tree.getChain(arm.base_link, arm.tip_link, chain)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from base='%s' to tip='%s'. Set base_link/tip_link correctly.", arm.base_link.c_str(), arm.tip_link.c_str());
        return false;
    }

    std::vector<std::string> raw_kdl_joint_names;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const auto & joint = chain.getSegment(i).getJoint();
        if (joint.getType() != KDL::Joint::None) {
            raw_kdl_joint_names.push_back(joint.getName());
        }
    }

    std::vector<KdlActiveJointMapping> mapping;
    if (!buildDefaultNoidArmJointMapping(arm, raw_kdl_joint_names, mapping)) {
        return false;
    }
    if (mapping.size() != raw_kdl_joint_names.size() || mapping.size() != chain.getNrOfJoints()) {
        RCLCPP_ERROR(this->get_logger(), "%s: KDL joint/mapping size mismatch.", arm.side.c_str());
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(arm.kdl_mutex);
        arm.chain = chain;
        arm.raw_kdl_joint_names = raw_kdl_joint_names;
        arm.kdl_active_mapping = mapping;
        arm.fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(arm.chain);
        arm.jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(arm.chain);
    }
    arm.kinematics_ready.store(true);

    std::vector<std::string> map_strings;
    for (const auto & m : mapping) {
        std::ostringstream ss;
        ss << m.kdl_joint_name << "->" << m.active_joint_name << "*" << m.multiplier;
        if (std::abs(m.offset) > 1e-12) {
            ss << "+" << m.offset;
        }
        map_strings.push_back(ss.str());
    }
    std::ostringstream oss;
    for (size_t i = 0; i < map_strings.size(); ++i) {
        if (i > 0) {
        oss << ", ";
        }
        oss << map_strings[i];
    }

    RCLCPP_INFO(this->get_logger(), "KDL chain ready: base=%s tip=%s", arm.base_link.c_str(), arm.tip_link.c_str());
    RCLCPP_INFO(this->get_logger(), "KDL-to-active mapping=[%s]", oss.str().c_str());
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

    // ハンド電流値のみ生データ
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

    if (publish_wrench_debug && arm.wrench_pub) {
        publishWrenchDebug(arm, arm.filtered, arm.sensor_frame_id);
    }

    Wrench6 gated = arm.filtered;
    gated.fx = applyDeadband(gated.fx, arm.force_deadband);
    gated.fy = applyDeadband(gated.fy, arm.force_deadband);
    gated.fz = applyDeadband(gated.fz, arm.force_deadband);
    gated.mx = applyDeadband(gated.mx, arm.torque_deadband);
    gated.my = applyDeadband(gated.my, arm.torque_deadband);
    gated.mz = applyDeadband(gated.mz, arm.torque_deadband);

    const bool collision = norm3(gated.fx, gated.fy, gated.fz) >= arm.force_threshold || norm3(gated.mx, gated.my, gated.mz) >= arm.torque_threshold;
    if (!on_force_feedback_.load() || !collision) {
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
    if (global_enabled && right_has_reflection && right_tau != nullptr) {
        mergeArmCurrent(msg, right_arm_, *right_tau);
    } else {
        resetReflectionDelta(right_arm_);
    }

    if (global_enabled && left_has_reflection && left_tau != nullptr){
        mergeArmCurrent(msg, left_arm_, *left_tau);
    } else {
        resetReflectionDelta(left_arm_);
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
        RCLCPP_ERROR_THROTTLE( this->get_logger(), *get_clock(), 1000, "%s reflection vector/parameter size mismatch.", arm.side.c_str());
        resetReflectionDelta(arm);
        return;
    }

    const double max_step = std::max(0.0, arm.max_delta_rate_per_sec) * dt;
    for (size_t axis = 0; axis < tau.size(); ++axis) {
        const int tracer_index = static_cast<int>(arm.arm_joint_indices[axis]);
        if (tracer_index < 0 || tracer_index >= MOTOR_COUNT || isProtectedTracerIndex(tracer_index)) {
            continue;
        }

        const double tau_i = applyDeadband(tau[axis], arm.tau_deadband);
        double target_delta = arm.current_signs[axis] * arm.current_gains[axis] * tau_i;
        target_delta = std::clamp(target_delta, -static_cast<double>(arm.max_current_delta), static_cast<double>(arm.max_current_delta));

        double limited_delta = target_delta;
        if (max_step > 0.0) {
            const double diff = target_delta - arm.last_current_delta[axis];
            limited_delta = arm.last_current_delta[axis] + std::clamp(diff, -max_step, max_step);
        }
        arm.last_current_delta[axis] = limited_delta;

        limited_delta = std::abs(limited_delta); //正方向の電流値として扱う暫定対策
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

void WrenchController::fillNeutral(aero_controller_msgs::msg::Current & msg) const {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        setCurrentWord(msg, i, neutral_current);
    }
}

// Dynamixel用電流値変換
int WrenchController::convertCurrentValue(int current_val) {
    // valを20で割り、-127～127に制限
    int driver_current_val = current_val / 20;
    driver_current_val = std::clamp(driver_current_val, -127, 127);
    // 符号付き8bit値として格納
    const int8_t driver_current_val_hex = static_cast<int8_t>(driver_current_val);
    // 同じビット列を符号なし8bit値として解釈
    const uint8_t driver_current_val_hex_u = static_cast<uint8_t>(driver_current_val_hex);
    // 変換後の値を50倍
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
