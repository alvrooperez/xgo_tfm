#include "xgo_driver/serial.hpp"
#include <unistd.h>
#include <fcntl.h>                                                                                                     
#include <termios.h>
#include <optional>
#include <chrono>
#include <thread>
namespace xgo {

    SerialPort::~SerialPort(){
        close();
    }

    void SerialPort::close(){
        if (fd_!=-1){
            ::close(fd_);
            fd_=-1;
        }
    }

    bool SerialPort::open(const std::string& port_name , int baudrate ) {
        if (fd_ != -1) {
            close();
        }

        port_name_=port_name;
        baudrate_=baudrate;

        fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); // read and write, non-block

        if (fd_ < 0) {
            fd_ = -1;
            return false;
        }

        // Restore blocking mode so VMIN/VTIME timeouts apply
        fcntl(fd_, F_SETFL, 0);

        struct termios tty;                                                                                                                           
        if (tcgetattr(fd_, &tty) != 0) {                                                                                                              
            close();                                                                                                                                  
            return false; // Error                                                                             
        }                                                                                                                                             
                                                                                                                                                      
        // Configurating baudrate                                                                                     
        speed_t speed = B115200;                                                                                                                      
        if (baudrate == 9600)   speed = B9600;                                                                                                        
        else if (baudrate == 57600) speed = B57600;                                                                                                   
                                                                                                                   
                                                                                                                                                      
        cfsetispeed(&tty, speed); // Velocidad de entrada (RX)                                                                                        
        cfsetospeed(&tty, speed); // Velocidad de salida (TX)                                                                                         
                                                                                                                                                      
        // Raw mode
        cfmakeraw(&tty);
  
        // CREAD: 
        tty.c_cflag |= (CREAD | CLOCAL);
  
        // 6. Timeouts de lectura (en décimas de segundo)
        // VMIN = 0: No waiting bytes
        // VTIME = 5: 500ms
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;
  
        // Configurate hardware
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close();
            return false;
        }
  
        // Clean Buffer
        tcflush(fd_, TCIOFLUSH);
  
        return true;
    }

    bool SerialPort::send(std::span<const uint8_t> data){
        if (fd_==-1) return false;

        auto now = std::chrono::steady_clock::now();                                                                                                      
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_last_);                                                     
                                                                                                                                                        
        if (elapsed.count() < 1) {                                                                                                                                                                                                    
            std::this_thread::sleep_for(std::chrono::milliseconds(1) - elapsed);                                                                          
        }                                                                                                                                                 


        ssize_t bytes_written = ::write(fd_, data.data(), data.size());
        if (bytes_written != static_cast<ssize_t>(data.size())){
            return false;
        }
        t_last_ = std::chrono::steady_clock::now();
        return true;

        
    }

    std::optional<uint8_t> SerialPort::read(){
        if (fd_==-1) return std::nullopt;
        uint8_t byte{0};
        ssize_t bytes_read = ::read(fd_, &byte, 1);
        if (bytes_read == 1) {
            return byte;
        }
        return std::nullopt;
    }


}
