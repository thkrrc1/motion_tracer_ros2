#ifndef TRACER_COMMAND_HPP_
#define TRACER_COMMAND_HPP_

#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <mutex>
#include <queue>

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace boost::asio;

namespace tracer
{
namespace controller
{

class SerialCommunication
{
public:
    SerialCommunication();
    ~SerialCommunication();

    bool openPort(std::string port, unsigned int baud_rate);
    void closePort();

    void startAsyncReceive();

    bool getPacket(std::vector<uint8_t>& packet);

    void write(const std::vector<uint8_t>& data);

private:
    void doReceive();
    void parseBuffer(const std::vector<uint8_t>& data);

    std::mutex serial_mutex_;

    io_service io_;
    serial_port serial_;
    boost::thread io_thread_;

    std::vector<uint8_t> raw_buffer_;
    std::mutex mutex_;

    std::queue<std::vector<uint8_t>> packet_queue_;
};

class TracerCommand
{
public:
    TracerCommand();
    ~TracerCommand();

    bool port_open(std::string port, unsigned int baud_rate);
    void port_close();

    bool get_packet(std::vector<uint8_t>& packet);

    void send_current(const std::vector<uint16_t>& currents);

private:
    SerialCommunication serial_com_;
    bool is_open_;
};


class FootPedalCommunication
{
public:
    FootPedalCommunication();
    ~FootPedalCommunication();

    bool openDevice(const std::string& device_path, bool grab_device);
    void closeDevice();
    bool getInput(std::string& input);
    bool getLatestInput(std::string& input);

private:
    bool openOnce();
    void closeFileDescriptor();
    void receiveLoop();
    void readAvailableEvents();
    void handleInputEvent(uint16_t type, uint16_t code, int32_t value);
    std::string keyCodeToOutput(uint16_t code) const;
    void pushInput(const std::string& input);

    int fd_;
    std::atomic<bool> running_;
    std::thread receive_thread_;

    std::string device_path_;
    bool grab_device_;
    bool store_on_repeat_;
    bool store_on_release_;
    std::string output_a_;
    std::string output_b_;
    std::string output_c_;
    std::string release_suffix_;
    std::string repeat_suffix_;
    int reopen_interval_ms_;
    int poll_timeout_ms_;

    mutable std::mutex input_mutex_;
    std::queue<std::string> input_queue_;
    std::string latest_input_;
};

class FootPedalCommand
{
public:
    FootPedalCommand();
    ~FootPedalCommand();

    bool device_open(const std::string& device_path, bool grab_device = true);
    void device_close();
    bool get_input(std::string& input);
    bool get_latest_input(std::string& input);

private:
    FootPedalCommunication foot_pedal_com_;
    bool is_open_;
};

} // namespace controller
} // namespace tracer

#endif