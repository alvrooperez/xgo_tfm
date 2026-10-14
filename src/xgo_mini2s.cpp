#include "xgo_driver/xgo_mini2s.hpp"
#include <algorithm>
#include <cmath>
#include <span>
#include <cstring>
#include <vector>
#include <chrono>



namespace xgo {
  
    uint8_t conver2u8(double value, double min, double max) {
        if (max <= min) {
            return 0;
        }
        const double clamped = std::clamp(value, min, max);
        return static_cast<uint8_t>(std::round(255.0 * (clamped - min) / (max - min)));
    }

    double conver2double(uint8_t value, double min, double max){
        return static_cast<double> (min+((max-min)/255.0)*value);
    }

    int16_t byte2short(uint8_t msb, uint8_t lsb) {
        const uint16_t combined = static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
        return static_cast<int16_t>(combined);
    }

    float Byte2float(std::span<const uint8_t, 4> data){
        float result{0.0f};
        std::memcpy(&result, data.data(), sizeof(result));
        return result;
    }

    uint8_t calculate_checksum(std::span<const uint8_t> data) {
        uint8_t sum {0};
        for (const auto byte:data){
            sum+=byte;
        }
        return static_cast<uint8_t>(~sum);
    }

    std::vector <uint8_t> create_write_packet(Register reg, std::span<const uint8_t> data){
        std::vector <uint8_t> packet ;
        packet.reserve(8+data.size());

        // Header
        packet.push_back(0x55);
        packet.push_back(0x00);
        //Lenght
        packet.push_back(static_cast<uint8_t>(8+data.size()));
        // Mode and Register
        packet.push_back(0x00);
        packet.push_back(static_cast<uint8_t>(reg));
        //Data
        packet.insert(packet.end(), data.begin(), data.end());
        //Checksum
        auto bytes = std::span(packet).subspan(2);

        packet.push_back(calculate_checksum(bytes));
        // Tail
        packet.push_back(0x00);
        packet.push_back(0xAA);

        return packet;

    }

    std::vector <uint8_t> create_read_packet(Register reg, uint8_t read_len){
        std::vector <uint8_t> packet ;
        packet.reserve(8+read_len);

        // Header
        packet.push_back(0x55);
        packet.push_back(0x00);
        //Lenght
        packet.push_back(static_cast<uint8_t>(9));
        // Mode and Register
        packet.push_back(0x02);
        packet.push_back(static_cast<uint8_t>(reg));
        //Data
        packet.push_back(read_len);
        //Checksum
        auto bytes = std::span(packet).subspan(2);

        packet.push_back(calculate_checksum(std::move(bytes)));
        // Tail
        packet.push_back(0x00);
        packet.push_back(0xAA);

        return packet;

    }

    std::optional<RxPacket> PacketParser::parse_byte(uint8_t byte){

        switch (state_) {                                                                                                                                 
            case RxState::HEADER_1:                                                                                                                       
                if (byte == 0x55) state_ = RxState::HEADER_2;                                                                                             
                break;                                                                                                                                    
                                                                                                                                                        
            case RxState::HEADER_2:                                                                                                                       
                if (byte == 0x00) {
                    state_ = RxState::LENGTH;
                } else if (byte == 0x55) {
                    state_ = RxState::HEADER_2; 
                } else {
                    reset();
                }
                break;

            case RxState::LENGTH: 
                length_=byte;                                                                                                                     
                state_ = RxState::MODE;                                                                                             
                break;                                                                                                                                    
                                                                                                                                                        
            case RxState::MODE:                                                                                                                       
                mode_ = byte;
                state_ = RxState::ADDR;
                break;                                                                                         
                 
            
            case RxState::ADDR:                                                                                                                       
                addr_=byte;
                state_ = RxState::DATA;                                                                                                
                break;                                                                                                                                    
                                                                                                                                                        
            case RxState::DATA:   
                payload_.push_back(byte);  

                if (payload_.size() >= static_cast<size_t>(length_ - 8)) {                                                                                                           
                    state_ = RxState::CHECKSUM;                                                                                                                   
                } 
                                                                                                       
                break; 
                                                                                                                                                   
            case RxState::CHECKSUM: { 
                std::vector<uint8_t> bytes_to_check;
                bytes_to_check.reserve(3 + payload_.size());
                bytes_to_check.push_back(length_);
                bytes_to_check.push_back(mode_);
                bytes_to_check.push_back(addr_);
                bytes_to_check.insert(bytes_to_check.end(), payload_.begin(), payload_.end()); 

                if (byte == calculate_checksum(bytes_to_check)) {
                    state_ = RxState::END_1;
                }else {
                    reset();
                }
                break;                                                                                              
            }                                                                                                                                    
                                                                                                                                                        
            case RxState::END_1:                                                                                                                       
                if (byte == 0x00){
                    state_ = RxState::END_2; 
                }else {
                    reset();
                }
                break;                                                                                        
                 
            
            case RxState::END_2:                                                                                                                       
                if (byte == 0xAA) {
                state_ = RxState::HEADER_1;
                RxPacket packet {static_cast<Register>(addr_),std::move(payload_)};
                reset();
                return packet;

                } else {
                    reset();
                }
                break; 
                                                                                                                                                 
        }

        return std::nullopt;
    }

    void PacketParser::reset() {                                                                                                                      
        state_ = RxState::HEADER_1;                                                                                                                   
        length_ = 0;                                                                                                                                  
        mode_ = 0;
        addr_ = 0;
        payload_.clear();
    }

    XgoMini2s::XgoMini2s() {
    }
  
    XgoMini2s::~XgoMini2s() {
        disconnect();
    }


    bool XgoMini2s::connect(const std::string& port , int baudrate ){
        
        return serial_.open(port,baudrate);
    }

    void XgoMini2s::disconnect(){
        stop();
        serial_.close();

    }

    // Movement
    bool XgoMini2s::set_velocity(double vx, double vy, double vyaw){
        if (serial_.is_open()==false) return false;

        uint8_t u_vx = conver2u8(vx, Limits::VX_MIN, Limits::VX_MAX);
        uint8_t u_vy = conver2u8(vy, Limits::VY_MIN, Limits::VY_MAX);
        uint8_t u_vyaw = conver2u8(vyaw, Limits::VYAW_MIN, Limits::VYAW_MAX);

        bool ok_x = serial_.send(create_write_packet(Register::VX,u_vx));
        bool ok_y = serial_.send(create_write_packet(Register::VY,u_vy));
        bool ok_yaw = serial_.send(create_write_packet(Register::VYAW,u_vyaw));

        return ok_x && ok_y && ok_yaw;

    }                                                                                         
    bool XgoMini2s::stop(){
        return set_velocity(0.0,0.0,0.0);
    }                                                                                                                                  
    bool XgoMini2s::reset(){
        if (!serial_.is_open()) return false;
        stop(); 
        return serial_.send(create_write_packet(Register::ACTION, static_cast<uint8_t>(255)));

    }                                                                                                        
    std::optional<RxPacket> XgoMini2s::query_register(Register reg, uint8_t read_len) {                                                               
        if (!serial_.is_open()) return std::nullopt; 
        parser_.reset();

        auto tx = create_read_packet(reg, read_len);
        if (!serial_.send(tx)) {                                                                                                                      
            return std::nullopt;                                                                                                                      
        }

        auto start = std::chrono::steady_clock::now();
        // Wait up to 500ms for incoming response packet
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - start).count() < 500) {
            if (auto byte = serial_.read()) {
                if (auto packet = parser_.parse_byte(*byte)) {
                    if (packet->reg == reg) {
                        return packet;
                    }
                }
            }
        }

        return std::nullopt;
    }

    // Telemetry                                                                                                                                
    std::optional<uint8_t> XgoMini2s::read_battery() {
        auto res = query_register(Register::BATTERY, 1);
        if (res && !res->payload.empty()) {
            return res->payload[0];
        }
        return std::nullopt;
    }                                                                                                       
    std::optional<std::string> XgoMini2s::read_firmware(){
        auto res = query_register(Register::FIRMWARE_VERSION, 10);
        if (res && !res->payload.empty()) {
            return std::string(res->payload.begin(), res->payload.end());
        }
        return std::nullopt;
    }                                                                                                  
    std::optional<ImuData> XgoMini2s::read_imu() {
        auto r_roll  = query_register(Register::ROLL, 4);
        auto r_pitch = query_register(Register::PITCH, 4);
        auto r_yaw   = query_register(Register::YAW, 4);

        if (r_roll && r_pitch && r_yaw &&
        r_roll->payload.size() >= 4 &&
        r_pitch->payload.size() >= 4 &&
        r_yaw->payload.size() >= 4) {
  
            ImuData imu;
            // Conversion to float
            imu.roll  = Byte2float(std::span<const uint8_t, 4>(r_roll->payload.data(), 4));
            imu.pitch = Byte2float(std::span<const uint8_t, 4>(r_pitch->payload.data(), 4));
            imu.yaw   = Byte2float(std::span<const uint8_t, 4>(r_yaw->payload.data(), 4));
    
            return imu;
        }
    
        return std::nullopt;

        }


}