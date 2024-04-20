#include "crobot/message/response.h"
#include "crobot/message/utils.h"

namespace crobot {

Response::Response(const std::vector<uint8_t>& data, uint32_t len)
    : data_(data) {
}

Message_Type Response::type() const {
    if (data_[3] >= static_cast<int>(Message_Type::MESSAGE_TYPE_MAX))
        return Message_Type::MESSAGE_TYPE_MAX;
    return static_cast<Message_Type>(data_[3]);
}

Get_Odometry_Resp Response::get_odometry_resp() const {
    Get_Odometry_Resp resp;
    resp.linear_x = hex_to_float(data_, 4);
    resp.linear_y = hex_to_float(data_, 8);
    resp.angular_z = hex_to_float(data_, 12);
    resp.position_x = hex_to_float(data_, 16);
    resp.position_y = hex_to_float(data_, 20);
    resp.direction = hex_to_float(data_, 24);

    return resp;
}

Get_IMU_Temperature_Resp Response::get_imu_temperature_resp() const {
    Get_IMU_Temperature_Resp resp;
    resp.temperature = hex_to_float(data_, 4);

    return resp;
}

Get_IMU_Data_Resp Response::get_imu_resp() const {
    Get_IMU_Data_Resp resp;
    resp.accel_x = hex_to_float(data_, 4);
    resp.accel_y = hex_to_float(data_, 8);
    resp.accel_z = hex_to_float(data_, 12);
    resp.angular_x = hex_to_float(data_, 16);
    resp.angular_y = hex_to_float(data_, 20);
    resp.angular_z = hex_to_float(data_, 24);

    return resp;
}

Get_Ultrasonic_Range_Resp Response::get_ultrasonic_range_resp() const {
    Get_Ultrasonic_Range_Resp resp;
    resp.range = (data_[4] << 8) | data_[5];

    return resp;
}

Get_Battery_Voltage_Resp Response::get_battery_voltage_resp() const {
    Get_Battery_Voltage_Resp resp;
    resp.voltage = hex_to_float(data_, 4);

    return resp;
}

} // namespace crobot
