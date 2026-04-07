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
    // ヘッダ検出
        if (raw_buffer_[0] == 0xFD && raw_buffer_[1] == 0xDF) {
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
        } else if (raw_buffer_[0] == 0xFB && raw_buffer_[1] == 0xBF) {
            if (raw_buffer_.size() < 7){
                return;
            }

            uint8_t len = raw_buffer_[2]+4;

            std::vector<uint8_t> packet(raw_buffer_.begin(), raw_buffer_.begin() + len);
            packet_queue_.push(packet);

            raw_buffer_.erase(raw_buffer_.begin(), raw_buffer_.begin() + 7);
        } else {
            // ゴミ除去
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

} // namespace controller
} // namespace tracer