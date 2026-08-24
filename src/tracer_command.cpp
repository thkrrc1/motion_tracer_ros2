#include "motion_tracer_ros2/tracer_command.hpp"

namespace tracer
{
namespace controller
{

SerialCommunication::SerialCommunication(): 
    serial_(io_) {
}

SerialCommunication::~SerialCommunication() {
    closePort();
}

bool SerialCommunication::openPort(std::string port, unsigned int baud_rate) {
    boost::system::error_code ec;
    serial_.open(port, ec);
    if (ec) return false;

    serial_.set_option(serial_port_base::baud_rate(baud_rate));

    startAsyncReceive();

    io_thread_ = boost::thread([this]() {
        io_.run();
    });

    return true;
}

void SerialCommunication::closePort() {
    if (serial_.is_open()) {
        serial_.close();
    }
    io_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

void SerialCommunication::startAsyncReceive() {
    doReceive();
}

void SerialCommunication::doReceive() {
    auto buf = std::make_shared<std::vector<uint8_t>>(128);

    serial_.async_read_some(
        buffer(*buf),
        [this, buf](boost::system::error_code ec, size_t length)
        {
            if (!ec) {
                std::vector<uint8_t> data(buf->begin(), buf->begin() + length);
                {
                  std::lock_guard<std::mutex> lock(mutex_);
                  raw_buffer_.insert(raw_buffer_.end(), data.begin(), data.end());
                }
                parseBuffer(data);
                doReceive();
            }
        });
}

void SerialCommunication::parseBuffer(const std::vector<uint8_t>&) {
    std::lock_guard<std::mutex> lock(mutex_);

    while (raw_buffer_.size() >= 4) {
    // Header detection
        if (raw_buffer_[0] == 0xDF && raw_buffer_[1] == 0xFD) {
            if (raw_buffer_.size() < 4) {
                return;
            }

            uint8_t len = raw_buffer_[2]+4;

            if (raw_buffer_.size() < len) {
                return;
            }

            std::vector<uint8_t> packet(raw_buffer_.begin(), raw_buffer_.begin() + len);
            packet_queue_.push(packet);

            raw_buffer_.erase(raw_buffer_.begin(), raw_buffer_.begin() + len);
        } else {
            // Garbage removal
            raw_buffer_.erase(raw_buffer_.begin());
        }
    }
}

bool SerialCommunication::getPacket(std::vector<uint8_t>& packet) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (packet_queue_.empty()) {
        return false;
    }
    packet = packet_queue_.front();
    packet_queue_.pop();
    return true;
}

void SerialCommunication::write(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    boost::system::error_code ec;

    boost::asio::write(
        serial_,
        boost::asio::buffer(data),
        ec
    );

    if (ec) {
        std::cerr << "serial write error: "
                  << ec.message() << std::endl;
    }
}


TracerCommand::TracerCommand() : 
    is_open_(false) {
}

TracerCommand::~TracerCommand() {
    port_close();
}

bool TracerCommand::port_open(std::string port, unsigned int baud_rate) {
    is_open_ = serial_com_.openPort(port, baud_rate);
    return is_open_;
}

void TracerCommand::port_close() {
    serial_com_.closePort();
    is_open_ = false;
}

bool TracerCommand::get_packet(std::vector<uint8_t>& packet) {
    return serial_com_.getPacket(packet);
}

void TracerCommand::send_current(const std::vector<uint16_t>& currents) {
    std::vector<uint8_t> packet;

    packet.push_back(0xFD);
    packet.push_back(0xDF);

    uint8_t len = 64;
    packet.push_back(len);

    // Current value transmission command
    uint8_t cmd = 0x01;
    packet.push_back(cmd);

    uint8_t mcid = 0x00;
    packet.push_back(mcid);

    for (auto c : currents) {
        packet.push_back(static_cast<uint8_t>(c >> 8));
        packet.push_back(static_cast<uint8_t>(c));
    }

    packet.push_back(0x7F);
    packet.push_back(0xFF);

    // checksum
    unsigned int cs = 0;
    cs += len;
    cs += cmd;
    cs += mcid;
    for (auto c : currents) {
        cs += static_cast<uint8_t>(c >> 8);
        cs += static_cast<uint8_t>(c);
    }
    cs += 0x7F;
    cs += 0xFF;
    packet.push_back(static_cast<uint8_t>(~cs));

    // write
    serial_com_.write(packet);
}



FootPedalCommunication::FootPedalCommunication() :
    fd_(-1), running_(false), grab_device_(true){
}

FootPedalCommunication::~FootPedalCommunication() {
    closeDevice();
}

bool FootPedalCommunication::openDevice(const std::string& device_path, bool grab_device) {
    if (running_.load()) {
        return true;
    }
    device_path_ = device_path;
    grab_device_ = grab_device;
    running_.store(true);
    receive_thread_ = std::thread(&FootPedalCommunication::receiveLoop, this);
    return true;
}

void FootPedalCommunication::closeDevice() {
    running_.store(false);

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    closeFileDescriptor();
}

bool FootPedalCommunication::getInput(std::string& input) {
    std::lock_guard<std::mutex> lock(input_mutex_);

    if (input_queue_.empty()) {
        return false;
    }

    input = input_queue_.front();
    input_queue_.pop();
    return true;
}

bool FootPedalCommunication::getLatestInput(std::string& input) {
    std::lock_guard<std::mutex> lock(input_mutex_);

    if (latest_input_.empty()) {
        return false;
    }

    input = latest_input_;
    return true;
}

bool FootPedalCommunication::openOnce() {
    fd_ = ::open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    if (grab_device_) {
        if (::ioctl(fd_, EVIOCGRAB, 1) < 0) {
            std::cerr << "EVIOCGRAB failed for foot pedal: "
                      << std::strerror(errno) << std::endl;
        }
    }

    return true;
}

void FootPedalCommunication::closeFileDescriptor() {
    if (fd_ >= 0) {
        if (grab_device_) {
            (void)::ioctl(fd_, EVIOCGRAB, 0);
        }
        ::close(fd_);
        fd_ = -1;
    }
}

void FootPedalCommunication::receiveLoop() {
    while (running_.load()) {
        if (fd_ < 0) {
            if (!openOnce()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }

        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;

        const int poll_ret = ::poll(&pfd, 1, 100);

        if (!running_.load()) {
            break;
        }

        if (poll_ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            closeFileDescriptor();
            continue;
        }

        if (poll_ret == 0) {
            continue;
        }

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            closeFileDescriptor();
            continue;
        }

        if (pfd.revents & POLLIN) {
            readAvailableEvents();
        }
    }

    closeFileDescriptor();
}

void FootPedalCommunication::readAvailableEvents() {
    while (running_.load()) {
        input_event ev{};
        const ssize_t n = ::read(fd_, &ev, sizeof(ev));

        if (n == static_cast<ssize_t>(sizeof(ev))) {
            handleInputEvent(ev.type, ev.code, ev.value);
            continue;
        }

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            if (errno == EINTR) {
                continue;
            }
            closeFileDescriptor();
            break;
        }

        if (n == 0) {
            closeFileDescriptor();
            break;
        }

        break;
    }
}

void FootPedalCommunication::handleInputEvent(uint16_t type, uint16_t code, int32_t value) {
    if (type != EV_KEY) {
        return;
    }

    const std::string base = keyCodeToOutput(code);
    if (base.empty()) {
        return;
    }

    // Linux input EV_KEY values:
    //   0: release, 1: press, 2: auto-repeat
    if (value == 1) {
        pushInput(base);
    }
}

std::string FootPedalCommunication::keyCodeToOutput(uint16_t code) const {
    switch (code) {
        case KEY_A:
            return "a";
        case KEY_B:
            return "b";
        case KEY_C:
            return "c";
        default:
            return "";
    }
}

void FootPedalCommunication::pushInput(const std::string& input) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_input_ = input;
    input_queue_.push(input);
}


FootPedalCommand::FootPedalCommand() :
    is_open_(false) {
}

FootPedalCommand::~FootPedalCommand() {
    device_close();
}

bool FootPedalCommand::device_open(const std::string& device_path, bool grab_device) {
    is_open_ = foot_pedal_com_.openDevice(device_path, grab_device);
    return is_open_;
}

void FootPedalCommand::device_close() {
    foot_pedal_com_.closeDevice();
    is_open_ = false;
}

bool FootPedalCommand::get_input(std::string& input) {
    return foot_pedal_com_.getInput(input);
}

bool FootPedalCommand::get_latest_input(std::string& input) {
    return foot_pedal_com_.getLatestInput(input);
}

} // namespace controller
} // namespace tracer