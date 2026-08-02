#include "motion_tracer_ros2/wrench_controller.hpp"

WrenchController::WrenchController() : 
    Node("wrench_controller_node"), fd_(-1) {

    tip_T_sensor = makeSensorFrameFromParams();

    last_current_delta.assign(active_joint_names.size(), 0.0);
    last_current_publish_time = now();

    auto current_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto notify_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto robot_description_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    current_raw_sub_ = this->create_subscription<aero_controller_msgs::msg::Current>("/current_controller/current_raw", current_qos, std::bind(&WrenchController::currentRawCallback, this, std::placeholders::_1));
    current_pub_ = this->create_publisher<aero_controller_msgs::msg::Current>("/current_controller/current", current_qos);

    if (publish_wrench_debug) {
        wrench_sensor_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/force_sensor/filtered_wrench", sensor_qos);
    }
    if (publish_tau_debug) {
        tau_pub_ = create_publisher<sensor_msgs::msg::JointState>("/force_sensor/reflection_tau", sensor_qos);
    }

    robot_description_sub_ = create_subscription<std_msgs::msg::String>("/robot_description", robot_description_qos, std::bind(&WrenchController::robotDescriptionCallback, this, std::placeholders::_1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", sensor_qos, std::bind(&WrenchController::jointStateCallback, this, std::placeholders::_1));

    on_force_feedback_sub_ = create_subscription<std_msgs::msg::Bool>("/on_force_feedback", notify_qos, std::bind(&WrenchController::onForceFeedbackCallback, this, std::placeholders::_1));

    tare_srv_ = this->create_service<std_srvs::srv::Trigger>("/force_sensor/tare", std::bind(&WrenchController::tareCallback, this, std::placeholders::_1, std::placeholders::_2));

    if (!openSerial("/dev/force_sensor")) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/force_sensor");
    }
    flushSerial();

    for (size_t i = 0; i < retry_calib_max_count; ++i) {
        if (!readCalibration()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read force sensor calibration.");
            continue;
        }
        break;
    }

    if (acquisition_mode == "continuous") {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (!startStream()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start force sensor data streaming.");
        }
    }
    // if (!setFrequencyDivider(frequency_divider_)) {
    //     RCLCPP_WARN(this->get_logger(), "Failed to set Dynpick frequency divider. Continue anyway.");
    // }

    publishNeutralCurrent();
    running_.store(true);
    wrench_thread_ = std::thread(&WrenchController::wrenchLoop, this);
}

WrenchController::~WrenchController() {
    running_.store(false);
    if (wrench_thread_.joinable()) {
        wrench_thread_.join();
    }
    if (acquisition_mode == "continuous") {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        stopStream();
    }
    publishNeutralCurrent();
    closeSerial();
}

void WrenchController::currentRawCallback(const aero_controller_msgs::msg::Current::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(current_raw_mutex);
    latest_current_raw = *msg;
    latest_current_raw_stamp = this->now();
    have_current_raw.store(true);
}

void WrenchController::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (kinematics_ready.load()) {
        return;
    }
    if (!msg || msg->data.empty()) {
        return;
    }
    buildKinematicsFromUrdf(msg->data);
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
    if (!msg.data) {
        resetReflectionDelta();
        publishPassThroughOrNeutral();
    }
}

void WrenchController::tareCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    tare_requested_.store(true);
    filter_initialized = false;
    res->success = true;
    res->message = "force sensor tare requested.";
}

bool WrenchController::openSerial(std::string device_) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    struct termios term;
    tcgetattr(fd_, &term);
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

    tcsetattr(fd_, TCSANOW, &term);

    return true;
}

void WrenchController::closeSerial() {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

void WrenchController::flushSerial() {
    continuous_rx_buffer_.clear();
    if (fd_ >= 0) {
        tcflush(fd_, TCIFLUSH);
    }
}

bool WrenchController::writeAll(const char * data, size_t len) {
    size_t written = 0;
    while (written < len) {
        const ssize_t n = write(fd_, data + written, len - written);
        if (n > 0) {
            written += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            continue;
        }
        return false;
    }
    if (fd_ >= 0) {
        tcdrain(fd_);
    }
    return true;
}

bool WrenchController::readCalibText(std::string& line, int timeout_ms, size_t max_len) {
    line.clear();

    if (fd_ < 0) {
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
        pfd.fd = fd_;
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
        const ssize_t n = read(fd_, &ch, 1);
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

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            return false;
        }
        if (n == 0) {
            return false;
        }
    }
}

bool WrenchController::readCalibration() {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (fd_ < 0) {
        return false;
    }
    flushSerial();

    if (!writeAll("E", 1)) {
        RCLCPP_WARN(this->get_logger(), "Failed to send stop stream command.");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!writeAll("p", 1)) {
        return false;
    }

    std::string reply;
    if (!readCalibText(reply, 100, 128)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read force sensor calibration reply.");
        return false;
    }

    double c[6];
    const int n = std::sscanf(reply.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf", &c[0], &c[1], &c[2], &c[3], &c[4], &c[5]);
    if (n != 6) {
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        calib_[i] = std::max(1e-9, static_cast<double>(c[i]));
    }
    RCLCPP_INFO(this->get_logger(), "force sensor calib LSB: %.3f %.3f %.3f %.3f %.3f %.3f",
                calib_[0], calib_[1], calib_[2], calib_[3], calib_[4], calib_[5]);
    flushSerial();
    return true;
}

bool WrenchController::startStream() {
    if (fd_ < 0) {
        return false;
    }
    flushSerial();

    for (size_t i = 0; i < 3; ++i) {
        if (!writeAll("O", 1)){
            return false;
        };
    }

    if (!writeAll("S", 1)) {
        return false;
    }

    continuous_output_started_.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    flushSerial();
    return true;
}

bool WrenchController::stopStream() {
    if (fd_ < 0) {
        return false;
    }
    if (!continuous_output_started_.load()) {
        return true;
    }

    if (!writeAll("E", 1)) {
        RCLCPP_WARN(this->get_logger(), "Failed to send stop stream command.");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    continuous_output_started_.store(false);
    flushSerial();
    return true;
}

bool WrenchController::handleTare() {
    if (fd_ < 0) {
        return false;
    }

    const bool was_continuous = acquisition_mode == "continuous" && continuous_output_started_.load();
    if (was_continuous) {
        stopStream();
    }

    flushSerial();
    for (size_t i = 0; i < 3; ++i) {
        if (!writeAll("O", 1)){
            if (was_continuous) {
                startStream();
            }
            return false;
        };
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    flushSerial();
    filter_initialized = false;

    if (was_continuous) {
        startStream();
    }
    return false;  // Do not use a sample in the same cycle as tare.
}

// bool WrenchController::setFrequencyDivider(int div) {
//     if (!(div == 1 || div == 2 || div == 4 || div == 8)) {
//         return false;
//     }
//     flushSerial();
//     std::lock_guard<std::mutex> lock(serial_mutex_);
//     char cmd[3]{};
//     std::snprintf(cmd, sizeof(cmd), "%dF", div);
//     const bool ok = writeAll(cmd, 2);
//     flushSerial();
//     return ok;
// }

bool WrenchController::readExact(uint8_t *dst, size_t len, int timeout_ms) {
    if (fd_ < 0 || dst == nullptr) {
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
        pfd.fd = fd_;
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

        const ssize_t n = read(fd_, dst + total, len - total);
        if (n > 0) {
            total += static_cast<size_t>(n);
            continue;
        }else if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}

bool WrenchController::readSampleRequestResponse(Wrench6 & out) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (fd_ < 0) {
        return false;
    }

    if (tare_requested_.exchange(false)) {
        return handleTare();
    }

    flushSerial();
    if (!writeAll("R", 1)) {
        return false;
    }

    std::array<uint8_t, WRENCH_DATA_LENGTH> frame;
    if (!readExact(frame.data(), frame.size(), 2)) {
        flushSerial();
        return false;
    }
    return parseFrame(frame.data(), out);
}

bool WrenchController::extractLatestFrame(std::array<uint8_t, WRENCH_DATA_LENGTH> & latest_frame) {
    bool found = false;
    size_t erase_until = 0;

    for (size_t i = 0; i + 1 < continuous_rx_buffer_.size(); ++i) {
        if (continuous_rx_buffer_[i] == CR && continuous_rx_buffer_[i + 1] == LF) {
            if (i >= WRENCH_PAYLOAD_DATA_LENGTH) {
                const size_t frame_start = i - WRENCH_PAYLOAD_DATA_LENGTH;
                if (frame_start + WRENCH_DATA_LENGTH <= continuous_rx_buffer_.size()) {
                    std::copy_n(continuous_rx_buffer_.begin() + frame_start, WRENCH_DATA_LENGTH, latest_frame.begin());
                    found = true;
                }
            }
            erase_until = i + 2;
            ++i;
        }
    }

    if (erase_until > 0) {
        continuous_rx_buffer_.erase(continuous_rx_buffer_.begin(), continuous_rx_buffer_.begin() + erase_until);
    }
    return found;
}

bool WrenchController::readStream(int timeout_ms) {
    if (fd_ < 0) {
        return false;
    }

    pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int ret = ::poll(&pfd, 1, timeout_ms);

    if (ret < 0) {
        if (errno == EINTR) {
            return true;
        }
        return false;
    }
    if (ret == 0) {
        return false;
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        return false;
    }
    if (!(pfd.revents & POLLIN)) {
        return false;
    }

    bool got_any = false;
    uint8_t buf[512];
    while (true) {
        const ssize_t n = read(fd_, buf, sizeof(buf));
        if (n > 0) {
            got_any = true;
            continuous_rx_buffer_.insert(continuous_rx_buffer_.end(), buf, buf + n);
            const size_t max_buf = static_cast<size_t>(frame_rx_buffer_max_bytes);
            if (continuous_rx_buffer_.size() > max_buf) {
                const size_t keep = static_cast<size_t>(frame_rx_buffer_keep_bytes);
                if (continuous_rx_buffer_.size() > keep) {
                    continuous_rx_buffer_.erase(continuous_rx_buffer_.begin(), continuous_rx_buffer_.end() - keep);
                }
            }
            continue;
        }

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            return false;
        }
        break;
    }
    return got_any;
}

bool WrenchController::readSampleContinuous(Wrench6 & out) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (fd_ < 0) {
        return false;
    }

    if (tare_requested_.exchange(false)) {
        return handleTare();
    }

    if (!continuous_output_started_.load()) {
        if (!startStream()) {
            return false;
        }
    }

    if (!readStream(2)) {
        return false;
    }

    for (int i = 0; i < 10; ++i) {
        if (!readStream(0)) {
            break;
        }
    }

    std::array<uint8_t, WRENCH_DATA_LENGTH> frame;
    if (!extractLatestFrame(frame)) {
        return false;
    }
    return parseFrame(frame.data(), out);
}

bool WrenchController::readSample(Wrench6 & out) {
    if (acquisition_mode == "continuous") {
        return readSampleContinuous(out);
    }
    return readSampleRequestResponse(out);
}

bool WrenchController::parseFrame(const uint8_t* frame, Wrench6& out) {
    if (frame == nullptr) {
        return false;
    }
    if (frame[WRENCH_PAYLOAD_DATA_LENGTH] != CR || frame[WRENCH_PAYLOAD_DATA_LENGTH + 1] != LF) {
        return false;
    }
    char payload[WRENCH_PAYLOAD_DATA_LENGTH + 1];
    std::memcpy(payload, frame, WRENCH_PAYLOAD_DATA_LENGTH);
    payload[WRENCH_PAYLOAD_DATA_LENGTH] = '\0';
    return parseWrench(payload, out);
}

bool WrenchController::parseWrench(const char *payload, Wrench6& out) {
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

    out.fx = (static_cast<double>(raw[0]) - lbs_val) / calib_[0];
    out.fy = (static_cast<double>(raw[1]) - lbs_val) / calib_[1];
    out.fz = (static_cast<double>(raw[2]) - lbs_val) / calib_[2];
    out.mx = (static_cast<double>(raw[3]) - lbs_val) / calib_[3];
    out.my = (static_cast<double>(raw[4]) - lbs_val) / calib_[4];
    out.mz = (static_cast<double>(raw[5]) - lbs_val) / calib_[5];
    return true;
}

bool WrenchController::buildDefaultNoidRightArmJointMapping(const std::vector<std::string> & raw_kdl_joint_names, std::vector<KdlActiveJointMapping> & mapping) {
    mapping.clear();
    mapping.reserve(raw_kdl_joint_names.size());

    std::vector<bool> active_used(active_joint_names.size(), false);

    for (const auto & raw_name : raw_kdl_joint_names) {
        std::string active_name = raw_name;
        double multiplier = 1.0;
        double offset = 0.0;

        if (raw_name == "r_elbow_joint_mimic") {
            active_name = "r_elbow_joint";
            multiplier = -1.0;
        } else if (raw_name == "r_elbow_middle_joint") {
            active_name = "r_elbow_joint";
            multiplier = 0.591222;
        } else if (raw_name == "r_elbow_middle_joint_mimic") {
            active_name = "r_elbow_joint";
            multiplier = 0.408778;
        }

        const int active_index = findActiveJointIndex(active_name);
        if (active_index < 0) {
            RCLCPP_ERROR(get_logger(), "KDL joint '%s' maps to active joint '%s', but active joint is not listed. active=[%s]",
                raw_name.c_str(), active_name.c_str(), joinStrings(active_joint_names).c_str());
            return false;
        }

        active_used[static_cast<size_t>(active_index)] = true;
        mapping.push_back(KdlActiveJointMapping{raw_name, active_name, active_index, multiplier, offset});
    }

    std::vector<std::string> missing;
    for (size_t i = 0; i < active_joint_names.size(); ++i) {
        if (!active_used[i]) {
        missing.push_back(active_joint_names[i]);
        }
    }

    if (!missing.empty()) {
        RCLCPP_ERROR(get_logger(), "KDL chain does not contain active right-arm joints=[%s]. Check base_link='%s' and tip_link='%s'. raw_kdl=[%s]",
        joinStrings(missing).c_str(), base_link_.c_str(), tip_link_.c_str(), joinStrings(raw_kdl_joint_names).c_str());
        return false;
    }

    return true;
}

// bool WrenchController::buildConfiguredJointMapping(const std::vector<std::string> & raw_kdl_joint_names, std::vector<KdlActiveJointMapping> & mapping) {
//     mapping.clear();

//     if (configured_kdl_joint_names_.empty() || configured_active_joint_names_.empty()) {
//         return false;
//     }
//     if (configured_kdl_joint_names_.size() != configured_active_joint_names_.size()) {
//         RCLCPP_ERROR(get_logger(), "kdl_joint_names and kdl_active_joint_names must have the same length.");
//         return false;
//     }
//     if (!configured_kdl_to_active_multipliers_.empty() && configured_kdl_to_active_multipliers_.size() != configured_kdl_joint_names_.size()) {
//         RCLCPP_ERROR(get_logger(), "kdl_to_active_multipliers must be empty or have the same length as kdl_joint_names.");
//         return false;
//     }
//     if (!configured_kdl_to_active_offsets_.empty() && configured_kdl_to_active_offsets_.size() != configured_kdl_joint_names_.size()) {
//         RCLCPP_ERROR(get_logger(), "kdl_to_active_offsets must be empty or have the same length as kdl_joint_names.");
//         return false;
//     }

//     std::vector<bool> active_used(active_joint_names.size(), false);
//     mapping.reserve(raw_kdl_joint_names.size());

//     for (const auto & raw_name : raw_kdl_joint_names) {
//         const auto it = std::find(configured_kdl_joint_names_.begin(), configured_kdl_joint_names_.end(), raw_name);
//         if (it == configured_kdl_joint_names_.end()) {
//             RCLCPP_ERROR(get_logger(), "KDL joint '%s' has no configured mapping. Configure kdl_joint_names/kdl_active_joint_names or enable use_noid_right_arm_default_mapping.", raw_name.c_str());
//             return false;
//         }

//         const size_t map_index = static_cast<size_t>(std::distance(configured_kdl_joint_names_.begin(), it));
//         const std::string & active_name = configured_active_joint_names_[map_index];
//         const int active_index = findActiveJointIndex(active_name);
//         if (active_index < 0) {
//             RCLCPP_ERROR(get_logger(), "Configured active joint '%s' for KDL joint '%s' is not in active_joint_names=[%s].", active_name.c_str(), raw_name.c_str(), joinStrings(active_joint_names).c_str());
//             return false;
//         }

//         const double multiplier = configured_kdl_to_active_multipliers_.empty() ? 1.0 : configured_kdl_to_active_multipliers_[map_index];
//         const double offset = configured_kdl_to_active_offsets_.empty() ? 0.0 : configured_kdl_to_active_offsets_[map_index];

//         active_used[static_cast<size_t>(active_index)] = true;
//         mapping.push_back(KdlActiveJointMapping{raw_name, active_name, active_index, multiplier, offset});
//     }

//     std::vector<std::string> missing;
//     for (size_t i = 0; i < active_joint_names.size(); ++i) {
//         if (!active_used[i]) {
//         missing.push_back(active_joint_names[i]);
//         }
//     }
//     if (!missing.empty()) {
//         RCLCPP_ERROR(get_logger(), "Configured KDL mapping does not cover active joints=[%s].", joinStrings(missing).c_str());
//         return false;
//     }

//     return true;
// }

bool WrenchController::buildKinematicsFromUrdf(const std::string & robot_description) {
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_description, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse robot_description as KDL tree.");
        return false;
    }

    KDL::Chain chain;
    if (!tree.getChain(base_link_, tip_link_, chain)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from base='%s' to tip='%s'. Set base_link/tip_link correctly.", base_link_.c_str(), tip_link_.c_str());
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
    bool mapping_ok = false;
    mapping_ok = buildDefaultNoidRightArmJointMapping(raw_kdl_joint_names, mapping);
    // mapping_ok = buildConfiguredJointMapping(raw_kdl_joint_names, mapping);

    if (!mapping_ok) {
        return false;
    }
    if (raw_kdl_joint_names.size() != mapping.size()) {
        RCLCPP_ERROR(get_logger(), "Internal error: raw_kdl_joint_names and mapping sizes differ.");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(kdl_mutex);
        chain_ = chain;
        raw_kdl_joint_names_ = raw_kdl_joint_names;
        kdl_active_mapping_ = mapping;
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    }
    kinematics_ready.store(true);

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

    RCLCPP_INFO(this->get_logger(), "KDL chain ready: base=%s tip=%s", base_link_.c_str(), tip_link_.c_str());
    RCLCPP_INFO(get_logger(), "KDL-to-active mapping=[%s]", oss.str().c_str());
    return true;
}

bool WrenchController::getActiveJointPositions(std::vector<double> & active_q, std::vector<std::string> & names) {
    if (!kinematics_ready.load() || !have_joint_state.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(joint_mutex);
    if ((now() - latest_joint_stamp).seconds() > 0.1) {
        return false;
    }

    active_q.assign(active_joint_names.size(), 0.0);
    for (size_t i = 0; i < active_joint_names.size(); ++i) {
        const auto it = latest_joint_position.find(active_joint_names[i]);
        if (it == latest_joint_position.end()) {
            return false;
        }
        active_q[i] = it->second;
    }

    names = active_joint_names;
    return true;
}

bool WrenchController::getCurrentJointArray(KDL::JntArray & q, std::vector<std::string> & names) {
    std::vector<double> active_q;
    std::vector<std::string> active_names;
    if (!getActiveJointPositions(active_q, active_names)) {
        return false;
    }

    std::vector<KdlActiveJointMapping> mapping;
    std::vector<std::string> raw_names;
    {
        std::lock_guard<std::mutex> lock(kdl_mutex);
        mapping = kdl_active_mapping_;
        raw_names = raw_kdl_joint_names_;
    }

    if (mapping.empty() || mapping.size() != raw_names.size()) {
        return false;
    }

    q = KDL::JntArray(mapping.size());
    for (size_t i = 0; i < mapping.size(); ++i) {
        const auto & m = mapping[i];
        if (m.active_index < 0 || static_cast<size_t>(m.active_index) >= active_q.size()) {
            return false;
        }
        q(i) = m.multiplier * active_q[static_cast<size_t>(m.active_index)] + m.offset;
    }
    names = raw_names;
    return true;
}

int WrenchController::findActiveJointIndex(const std::string & name) {
    const auto it = std::find(active_joint_names.begin(), active_joint_names.end(), name);
    if (it == active_joint_names.end()) {
        return -1;
    }
    return static_cast<int>(std::distance(active_joint_names.begin(), it));
}

KDL::Frame WrenchController::makeSensorFrameFromParams() const {
    return KDL::Frame(
        KDL::Rotation::RPY(sensor_rpy_in_tip[0], sensor_rpy_in_tip[1], sensor_rpy_in_tip[2]),
        KDL::Vector(sensor_xyz_in_tip[0], sensor_xyz_in_tip[1], sensor_xyz_in_tip[2])
    );
}

KDL::Vector WrenchController::cross(const KDL::Vector & a, const KDL::Vector & b) {
    return KDL::Vector(
      a.y() * b.z() - a.z() * b.y(),
      a.z() * b.x() - a.x() * b.z(),
      a.x() * b.y() - a.y() * b.x());
}

bool WrenchController::computeTauFromWrench(const Wrench6 & sensor_wrench, std::vector<double> & tau_out) {
    KDL::JntArray q_raw;
    std::vector<std::string> raw_names;
    if (!getCurrentJointArray(q_raw, raw_names)) {
        return false;
    }

    KDL::Chain chain_copy;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;
    std::vector<KdlActiveJointMapping> mapping;
    std::vector<std::string> active_names;
    {
        std::lock_guard<std::mutex> lock(kdl_mutex);
        chain_copy = chain_;
        mapping = kdl_active_mapping_;
        active_names = active_joint_names;
        fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_copy);
        jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(chain_copy);
    }

    if (mapping.empty() || mapping.size() != q_raw.rows()) {
        return false;
    }

    KDL::Frame base_T_tip;
    if (fk->JntToCart(q_raw, base_T_tip) < 0) {
        return false;
    }

    KDL::Jacobian jac(q_raw.rows());
    if (jac_solver->JntToJac(q_raw, jac) < 0) {
        return false;
    }

    KDL::Vector f_sensor(sensor_wrench.fx, sensor_wrench.fy, sensor_wrench.fz);
    KDL::Vector m_sensor(sensor_wrench.mx, sensor_wrench.my, sensor_wrench.mz);

    KDL::Vector f_tip = tip_T_sensor.M * f_sensor;
    KDL::Vector m_tip = tip_T_sensor.M * m_sensor + cross(tip_T_sensor.p, f_tip);

    KDL::Vector f_base = base_T_tip.M * f_tip;
    KDL::Vector m_base = base_T_tip.M * m_tip;

    std::vector<double> tau_raw(jac.columns(), 0.0);
    for (unsigned int j = 0; j < jac.columns(); ++j) {
        tau_raw[j] =
        jac(0, j) * f_base.x() + jac(1, j) * f_base.y() + jac(2, j) * f_base.z() +
        jac(3, j) * m_base.x() + jac(4, j) * m_base.y() + jac(5, j) * m_base.z();
    }

    tau_out.assign(active_names.size(), 0.0);
    const size_t n = std::min(mapping.size(), tau_raw.size());
    for (size_t i = 0; i < n; ++i) {
        const auto & m = mapping[i];
        if (m.active_index < 0 || static_cast<size_t>(m.active_index) >= tau_out.size()) {
            continue;
        }
        tau_out[static_cast<size_t>(m.active_index)] += m.multiplier * tau_raw[i];
    }

    if (publish_tau_debug && tau_pub_) {
        publishTauDebug(tau_out, active_names);
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

        Wrench6 w;
        if (!readSample(w)) {
            publishPassThroughOrNeutral();
            std::this_thread::sleep_until(next);
            continue;
        }

        // std::cout<<"-------------Wrench6-------------"<<std::endl;
        // std::cout<<"fx : "<< w.fx <<std::endl;
        // std::cout<<"fy : "<< w.fy <<std::endl;
        // std::cout<<"fz : "<< w.fz <<std::endl;
        // std::cout<<"mx : "<< w.mx <<std::endl;
        // std::cout<<"my : "<< w.my <<std::endl;
        // std::cout<<"mz : "<< w.mz <<std::endl;
        // std::cout<<"---------------------------------"<<std::endl;

        if (!filter_initialized) {
            filtered_ = w;
            filter_initialized = true;
        } else {
            const double a = std::clamp(lowpass_alpha, 0.0, 1.0);
            filtered_.fx = a * w.fx + (1.0 - a) * filtered_.fx;
            filtered_.fy = a * w.fy + (1.0 - a) * filtered_.fy;
            filtered_.fz = a * w.fz + (1.0 - a) * filtered_.fz;
            filtered_.mx = a * w.mx + (1.0 - a) * filtered_.mx;
            filtered_.my = a * w.my + (1.0 - a) * filtered_.my;
            filtered_.mz = a * w.mz + (1.0 - a) * filtered_.mz;
        }

        Wrench6 gated = filtered_;
        gated.fx = applyDeadband(gated.fx, force_deadband);
        gated.fy = applyDeadband(gated.fy, force_deadband);
        gated.fz = applyDeadband(gated.fz, force_deadband);
        gated.mx = applyDeadband(gated.mx, torque_deadband);
        gated.my = applyDeadband(gated.my, torque_deadband);
        gated.mz = applyDeadband(gated.mz, torque_deadband);

        const double force_norm = norm3(gated.fx, gated.fy, gated.fz);
        const double torque_norm = norm3(gated.mx, gated.my, gated.mz);
        const bool collision = (force_norm >= force_threshold) || (torque_norm >= torque_threshold);

        if (publish_wrench_debug && wrench_sensor_pub_) {
            geometry_msgs::msg::WrenchStamped msg;
            msg.header.stamp = now();
            msg.header.frame_id = "force_sensor_base";
            msg.wrench.force.x = filtered_.fx;
            msg.wrench.force.y = filtered_.fy;
            msg.wrench.force.z = filtered_.fz;
            msg.wrench.torque.x = filtered_.mx;
            msg.wrench.torque.y = filtered_.my;
            msg.wrench.torque.z = filtered_.mz;
            wrench_sensor_pub_->publish(msg);
        }

        if (!on_force_feedback_.load() || !collision) {
            publishPassThroughOrNeutral();
            std::this_thread::sleep_until(next);
            continue;
        }

        std::vector<double> tau;
        if (!computeTauFromWrench(gated, tau)) {
            publishPassThroughOrNeutral();
            std::this_thread::sleep_until(next);
            continue;
        }

        publishMergedCurrent(&tau, true);
        std::this_thread::sleep_until(next);
    }
}

void WrenchController::resetReflectionDelta() {
    std::fill(last_current_delta.begin(), last_current_delta.end(), 0.0);
    last_current_publish_time = now();
}

void WrenchController::publishPassThroughOrNeutral() {
    resetReflectionDelta();
    auto msg = makeOutputFromBaseOrNeutral();
    current_pub_->publish(msg);
}

void WrenchController::publishNeutralCurrent() {
    resetReflectionDelta();
    aero_controller_msgs::msg::Current msg;
    fillNeutral(msg);

    std::lock_guard<std::mutex> lock(current_raw_mutex);
    if (have_current_raw.load()) {
        const size_t n_pos = std::min(msg.pos_data.size(), latest_current_raw.pos_data.size());
        std::copy_n(latest_current_raw.pos_data.begin(), n_pos, msg.pos_data.begin());
    }
    current_pub_->publish(msg);
}

void WrenchController::publishMergedCurrent(const std::vector<double> * tau, bool has_reflection) {
    auto msg = makeOutputFromBaseOrNeutral();

    if (!has_reflection || tau == nullptr || !on_force_feedback_.load()) {
        resetReflectionDelta();
        current_pub_->publish(msg);
        return;
    }

    const rclcpp::Time t = now();
    double dt = (t - last_current_publish_time).seconds();
    if (!(dt > 0.0) || dt > 1.0) {
        dt = 1.0 / std::max(1.0, sample_rate_hz);
    }
    last_current_publish_time = t;

    const size_t n = std::min({tau->size(), arm_joint_indices.size(), current_signs.size(), current_gains.size(), last_current_delta.size()});
    const double max_step = max_delta_rate_per_sec_ * dt;

    for (size_t axis = 0; axis < n; ++axis) {
        const int tracer_index = static_cast<int>(arm_joint_indices[axis]);
        if (tracer_index < 0 || tracer_index >= MOTOR_COUNT) {
            continue;
        }
        if (isProtectedTracerIndex(tracer_index)) {
            continue;
        }

        double tau_i = applyDeadband((*tau)[axis], tau_deadband);
        double target_delta = current_signs[axis] * current_gains[axis] * tau_i;
        target_delta = std::clamp(target_delta, -static_cast<double>(max_current_delta), static_cast<double>(max_current_delta));

        const double diff = target_delta - last_current_delta[axis];
        double limited_delta = target_delta;
        if (max_step > 0.0) {
            if (diff > max_step) {
                limited_delta = last_current_delta[axis] + max_step;
            } else if (diff < -max_step) {
                limited_delta = last_current_delta[axis] - max_step;
            }
        }
        last_current_delta[axis] = limited_delta;
        limited_delta = std::abs(limited_delta); //正方向の電流値として扱う
        const int merged_current_raw = static_cast<int>(std::llround(limited_delta));
        const int merged_current = convertCurrentValue(merged_current_raw);
        setCurrentWord(msg, tracer_index, clampCurrent(merged_current));
    }
    current_pub_->publish(msg);
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
    if (driver_current_val > 127) {
        driver_current_val = 127;
    } else if (driver_current_val < -127) {
        driver_current_val = -127;
    }

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

void WrenchController::publishTauDebug(const std::vector<double> & tau, const std::vector<std::string> & names) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = names;
    msg.effort = tau;
    tau_pub_->publish(msg);
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
