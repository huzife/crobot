#ifndef CROBOT_MESSAGE_RESPONSE_H
#define CROBOT_MESSAGE_RESPONSE_H

#include "type.h"
#include <cstdint>
#include <vector>

namespace crobot {

struct Get_Odometry_Resp {
    float linear_x;
    float linear_y;
    float angular_z;
    float position_x;
    float position_y;
    float direction;
};

struct Get_IMU_Temperature_Resp {
    float temperature;
};

struct Get_IMU_Data_Resp {
    float accel_x;
    float accel_y;
    float accel_z;
    float angular_x;
    float angular_y;
    float angular_z;
};

struct Get_Ultrasonic_Range_Resp {
    uint16_t range;
};

struct Get_Battery_Voltage_Resp {
    float voltage;
};

class Response {
private:
    std::vector<uint8_t> data_;

public:
    Response(const std::vector<uint8_t>& data, uint32_t len);

    Message_Type type() const;
    Get_Odometry_Resp get_odometry_resp() const;
    Get_IMU_Temperature_Resp get_imu_temperature_resp() const;
    Get_IMU_Data_Resp get_imu_resp() const;
    Get_Ultrasonic_Range_Resp get_ultrasonic_range_resp() const;
    Get_Battery_Voltage_Resp get_battery_voltage_resp() const;
};

} // namespace crobot

#endif // CROBOT_MESSAGE_RESPONSE_H
