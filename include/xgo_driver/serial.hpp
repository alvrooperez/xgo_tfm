// serial.hpp
#pragma once
#include <string>
#include <cstdint>
#include <span>
#include <optional> 
#include <chrono>    

namespace xgo {
class SerialPort {
    std::string port_name_{"/dev/ttyAMA0"};
    int baudrate_{115200};
    int fd_{-1};
    int timeout_ms=100;

    int min_t=1;
    std::chrono::steady_clock::time_point t_last_{};

public:
    SerialPort()=default;
    ~SerialPort();
    bool open(const std::string& port_name = "/dev/ttyAMA0", int baudrate = 115200);
    void close();

    bool send(std::span<const uint8_t> data);
    std::optional<uint8_t> read();
    
    bool is_open() const { return fd_ != -1; }


};

}
