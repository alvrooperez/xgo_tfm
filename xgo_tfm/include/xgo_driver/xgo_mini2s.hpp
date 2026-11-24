#pragma once
#include <cstdint> 
#include <span>
#include <vector>
#include <optional>
#include "xgo_driver/serial.hpp"

enum class Register : uint8_t {                                                                                                                                                  
    BATTERY          = 0x01,                                                                                                                                                     
    PERFORM          = 0x03,                                                                                                                                                     
    CALIBRATION      = 0x04,                                                                                                                                                     
    UPGRADE          = 0x05,                                                                                                                                                     
    SET_ORIGIN       = 0x06,                                                                                                                                                     
    FIRMWARE_VERSION = 0x07,                                                                                                                                                     
    GAIT_TYPE        = 0x09,                                                                                                                                                     
    BT_NAME          = 0x13,                                                                                                                                                     
                                                                                                                                                                                    
    // Motors                                                                                                                                                                 
    UNLOAD_MOTOR     = 0x20,                                                                                                                                                     
    LOAD_MOTOR       = 0x20,                                                                                                                                                     
    TEACH_RECORD     = 0x21,                                                                                                                                                     
    TEACH_PLAY       = 0x22,                                                                                                                                                     
    TEACH_ARM_RECORD = 0x23,                                                                                                                                                     
    TEACH_ARM_PLAY   = 0x24,                                                                                                                                                     
                                                                                                                                                                                    
    // Movement and velocities                                                                                                                                      
    VX               = 0x30,                                                                                                                                                     
    VY               = 0x31,                                                                                                                                                     
    VYAW             = 0x32,                                                                                                                                                     
    TRANSLATION      = 0x33,                                                                                                                                                     
    ATTITUDE         = 0x36,                                                                                                                                                     
    PERIODIC_ROT     = 0x39,                                                                                                                                                     
    MARK_TIME        = 0x3C,                                                                                                                                                     
    MOVE_MODE        = 0x3D,                                                                                                                                                     
    ACTION           = 0x3E,                                                                                                                                                     
    MOVE_TO          = 0x3F,                                                                                                                                                     
    PERIODIC_TRAN    = 0x80,                                                                                                                                                     
                                                                                                                                                                                    
    // joints                                                                                                                                      
    LEG_POS          = 0x40,                                                                                                                                                     
    MOTOR_ANGLE      = 0x50,                                                                                                                                                     
    MOTOR_SPEED      = 0x5C,                                                                                                                                                     
    MOVE_TO_MID      = 0x5F,                                                                                                                                                     
                                                                                                                                                                                    
    // Sensors                                                                                                                                                           
    IMU              = 0x61,                                                                                                                                                     
    ROLL             = 0x62,                                                                                                                                                     
    PITCH            = 0x63,                                                                                                                                                     
    YAW              = 0x64,                                                                                                                                                     
    LED_COLOR        = 0x69,                                                                                                                                                     
                                                                                                                                                                                    
    // Robotic Arm                                                                                                                                
    ARM_ROTATE       = 0x70,                                                                                                                                                     
    CLAW             = 0x71,                                                                                                                                                     
    ARM_MODE         = 0x72,                                                                                                                                                     
    ARM_X            = 0x73,                                                                                                                                                     
    ARM_Z            = 0x74,                                                                                                                                                     
    ARM_SPEED        = 0x75,                                                                                                                                                     
    ARM_THETA        = 0x76,                                                                                                                                                     
    ARM_R            = 0x77,                                                                                                                                                     
    
    // GPIOs
    OUTPUT_ANALOG    = 0x90,
    OUTPUT_DIGITAL   = 0x91,
    ANALOG_READ      = 0x92,
    DIGITAL_READ     = 0x93,
    
    // Dog 
    WHEEL_CTRL_FLAG  = 0xB0,
    WHEEL_SPEED      = 0xB1,
    WHEEL_ENCODER    = 0xB2
    };



namespace xgo {                                                                                                                                                                  
                                                                                                                                                                                     
                                                                                                                     
    uint8_t conver2u8(double value, double min, double max);    
                                                                                                                                                                                                                                                                 
    double conver2double(uint8_t data, double min, double max); 

    int16_t byte2short(uint8_t msb, uint8_t lsb);

    float Byte2float(std::span<const uint8_t, 4> data);

    uint8_t calculate_checksum(std::span<const uint8_t> data); 

    std::vector <uint8_t> create_write_packet(Register reg, std::span<const uint8_t> data);
    inline std::vector<uint8_t> create_write_packet(Register reg, uint8_t value) {
        return create_write_packet(reg, std::span(&value, 1));
    }

    std::vector <uint8_t> create_read_packet(Register reg, uint8_t read_len=1);

    enum class RxState {
        HEADER_1, HEADER_2, LENGTH, MODE, ADDR, DATA, CHECKSUM, END_1, END_2
    };
    struct RxPacket {
        Register reg;
        std::vector<uint8_t> payload;
    };

    class PacketParser {

        RxState state_{RxState::HEADER_1};
        uint8_t length_{0};
        uint8_t mode_{0};
        uint8_t addr_{0};
        std::vector<uint8_t> payload_{};
        

    public:
        PacketParser() = default;
  
        std::optional<RxPacket> parse_byte(uint8_t byte);

        void reset();
        
    };

    struct Limits {                                                                                                                                   
        //Lineal velocities                                                                                 
        static constexpr double VX_MIN = -0.25;                                                                                           
        static constexpr double VX_MAX =  0.25;                                                                                           
                                                                                                                                                      
        static constexpr double VY_MIN = -0.18;                                                                                            
        static constexpr double VY_MAX =  0.18;                                                                                            
                                                                                                                                                      
        // Angular velocities                                                                            
        // 100 * pi / 180 ≈ 1.745 rad/s                                                                                                               
        static constexpr double VYAW_MIN = -1.745;                                                                                                    
        static constexpr double VYAW_MAX =  1.745;                                                                                                    
  
        // Batery
        static constexpr uint8_t BATTERY_MIN = 0;
        static constexpr uint8_t BATTERY_MAX = 100;
    };

    struct ImuData {                                                                                                                                  
        float roll{0.0f};                                                                                        
        float pitch{0.0f};                                                                                      
        float yaw{0.0f};                                                                                       
    };

    class XgoMini2s {
        SerialPort serial_;                                                                              
        PacketParser parser_;
        std::optional<RxPacket> query_register(Register reg, uint8_t read_len);
    public:
        XgoMini2s();
        ~XgoMini2s();

        // Conectivity                                                                                                                  
        bool connect(const std::string& port = "/dev/ttyAMA0", int baudrate = 115200);                                                                
        void disconnect();                                                                                                                            
        bool is_connected() const {return serial_.is_open(); };                                                                                                                    
                                                                                                                                                      
        // Movement                                                                                      
        bool set_velocity(double vx, double vy, double vyaw);                                                                                         
        bool stop();                                                                                                                                  
        bool reset();                                                                                                        
                                                                                                                                                      
        // Telemetry                                                                                                                                
        std::optional<uint8_t> read_battery();                                                                                                        
        std::optional<std::string> read_firmware();                                                                                                   
        std::optional<ImuData> read_imu(); // Roll, Pitch, Yaw
    };
                                                                                                                                                                                     
}


