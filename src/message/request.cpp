#include "crobot/message/request.h"
#include "crobot/message/type.h"
#include "crobot/message/utils.h"

using namespace std;

namespace crobot {

Set_PID_Interval_Req::Set_PID_Interval_Req(uint16_t pid_interval)
    : pid_interval_(pid_interval) {}

vector<uint8_t> Set_PID_Interval_Req::data() const {
    vector<uint8_t> ret(6);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x03;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_PID_INTERVAL);
    ret[4] = pid_interval_ >> 8;
    ret[5] = pid_interval_ & 0xFF;

    return ret;
}

Set_Count_Per_Rev_Req::Set_Count_Per_Rev_Req(uint16_t cpr)
    : cpr_(cpr) {}

vector<uint8_t> Set_Count_Per_Rev_Req::data() const {
    vector<uint8_t> ret(6);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x03;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_COUNT_PER_REV);
    ret[4] = cpr_ >> 8;
    ret[5] = cpr_ & 0xFF;

    return ret;
}

Set_Correction_Factor_Req::Set_Correction_Factor_Req(float linear, float angular)
    : linear_(linear),
      angular_(angular) {}

vector<uint8_t> Set_Correction_Factor_Req::data() const {
    vector<uint8_t> ret(12);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x09;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_CORRECTION_FACTOR);
    float_to_hex(linear_, ret, 4);
    float_to_hex(angular_, ret, 8);

    return ret;
}

Set_Velocity_Req::Set_Velocity_Req(float linear_x, float linear_y, float angular_z)
    : linear_x_(linear_x),
      linear_y_(linear_y),
      angular_z_(angular_z) {}

vector<uint8_t> Set_Velocity_Req::data() const {
    vector<uint8_t> ret(16);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x0D;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_VELOCITY);
    float_to_hex(linear_x_, ret, 4);
    float_to_hex(linear_y_, ret, 8);
    float_to_hex(angular_z_, ret, 12);

    return ret;
}

vector<uint8_t> Get_Odometry_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x01;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_ODOMETRY);

    return ret;
}

vector<uint8_t> Reset_Odometry_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x01;
    ret[3] = static_cast<uint8_t>(Message_Type::RESET_ODOMETRY);

    return ret;
}

vector<uint8_t> Get_IMU_Temperature_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x01;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_IMU_TEMPERATURE);

    return ret;
}

vector<uint8_t> Get_IMU_Data_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x01;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_IMU_DATA);

    return ret;
}

vector<uint8_t> Get_Ultrasonic_Range_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x01;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_ULTRASONIC_RANGE);

    return ret;
}

vector<uint8_t> Get_Battery_Voltage_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x01;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_BATTERY_VOLTAGE);

    return ret;
}

} // namespace crobot
