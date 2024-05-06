#include "crobot/message/request.h"
#include "crobot/message/type.h"
#include "crobot/message/utils.h"
#include <cstring>

#include <iostream>
using namespace std;

namespace crobot {

Set_PID_Interval_Req::Set_PID_Interval_Req(uint16_t pid_interval)
    : pid_interval_(pid_interval) {}

vector<uint8_t> Set_PID_Interval_Req::data() const {
    vector<uint8_t> ret(6);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_PID_INTERVAL);
    ret[4] = pid_interval_ >> 8;
    ret[5] = pid_interval_ & 0xFF;

    return ret;
}

Set_Motor_Param_Req::Set_Motor_Param_Req(uint32_t cpr, bool reverse)
    : cpr_(cpr),
      reverse_(reverse) {}

vector<uint8_t> Set_Motor_Param_Req::data() const {
    vector<uint8_t> ret(9);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_MOTOR_PARAM);
    for (int i = 0; i < 4; i++) {
        ret[i + 4] = (cpr_ >> (8 * (3 - i))) & 0xFF;
    }
    ret[8] = reverse_;

    return ret;
}

Set_Robot_Base_2WD_Req::Set_Robot_Base_2WD_Req(const Robot_Base_2WD_Param& param)
    : param_(param) {}

vector<uint8_t> Set_Robot_Base_2WD_Req::data() const {
    vector<uint8_t> ret(5 + sizeof(param_));
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_ROBOT_BASE);
    ret[4] = static_cast<uint8_t>(Robot_Base_Type::ROBOT_BASE_2WD);
    memcpy(ret.data() + 5, &param_, sizeof(param_));

    return ret;
}

Set_Robot_Base_3WO_Req::Set_Robot_Base_3WO_Req(const Robot_Base_3WO_Param& param)
    : param_(param) {}

vector<uint8_t> Set_Robot_Base_3WO_Req::data() const {
    vector<uint8_t> ret(5 + sizeof(param_));
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_ROBOT_BASE);
    ret[4] = static_cast<uint8_t>(Robot_Base_Type::ROBOT_BASE_3WO);
    memcpy(ret.data() + 5, &param_, sizeof(param_));

    return ret;
}

Set_Robot_Base_4WD_Req::Set_Robot_Base_4WD_Req(const Robot_Base_4WD_Param& param)
    : param_(param) {}

vector<uint8_t> Set_Robot_Base_4WD_Req::data() const {
    vector<uint8_t> ret(5 + sizeof(param_));
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_ROBOT_BASE);
    ret[4] = static_cast<uint8_t>(Robot_Base_Type::ROBOT_BASE_4WD);
    memcpy(ret.data() + 5, &param_, sizeof(param_));

    return ret;
}

Set_Robot_Base_4MEC_Req::Set_Robot_Base_4MEC_Req(const Robot_Base_4MEC_Param& param)
    : param_(param) {}

vector<uint8_t> Set_Robot_Base_4MEC_Req::data() const {
    vector<uint8_t> ret(5 + sizeof(param_));
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_ROBOT_BASE);
    ret[4] = static_cast<uint8_t>(Robot_Base_Type::ROBOT_BASE_4MEC);
    memcpy(ret.data() + 5, &param_, sizeof(param_));

    return ret;
}

Set_Correction_Factor_Req::Set_Correction_Factor_Req(float linear_x, float linear_y, float angular)
    : linear_x_(linear_x),
      linear_y_(linear_y),
      angular_(angular) {}

vector<uint8_t> Set_Correction_Factor_Req::data() const {
    vector<uint8_t> ret(16);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_CORRECTION_FACTOR);
    float_to_hex(linear_x_, ret, 4);
    float_to_hex(linear_y_, ret, 8);
    float_to_hex(angular_, ret, 12);

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
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_VELOCITY);
    float_to_hex(linear_x_, ret, 4);
    float_to_hex(linear_y_, ret, 8);
    float_to_hex(angular_z_, ret, 12);

    return ret;
}

vector<uint8_t> Reset_Odometry_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::RESET_ODOMETRY);

    return ret;
}

vector<uint8_t> Get_Odometry_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_ODOMETRY);

    return ret;
}

vector<uint8_t> Get_IMU_Temperature_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_IMU_TEMPERATURE);

    return ret;
}

vector<uint8_t> Get_IMU_Data_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_IMU_DATA);

    return ret;
}

vector<uint8_t> Get_Ultrasonic_Range_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_ULTRASONIC_RANGE);

    return ret;
}

vector<uint8_t> Get_Battery_Voltage_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = ret.size() - 3;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_BATTERY_VOLTAGE);

    return ret;
}

} // namespace crobot
