#ifndef TRACER_COMMAND_HPP_
#define TRACER_COMMAND_HPP_

#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <mutex>
#include <queue>

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

} // namespace controller
} // namespace tracer

#endif