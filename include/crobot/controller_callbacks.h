#ifndef CROBOT_CONTROLLER_CALLBACK_H
#define CROBOT_CONTROLLER_CALLBACK_H

#include "crobot/message/response.h"

namespace crobot {

class Controller_Callbacks {
public:
    virtual void set_pid_interval_callback() = 0;

    virtual void set_count_per_rev_callback() = 0;

    virtual void set_correction_factor_callback() = 0;

    virtual void set_velocity_callback() = 0;

    virtual void get_odometry_callback(const Get_Odometry_Resp& resp) = 0;

    virtual void reset_odometry_callback() = 0;

    virtual void get_imu_temperature_callback(const Get_IMU_Temperature_Resp& resp) = 0;

    virtual void get_imu_data_callback(const Get_IMU_Data_Resp& resp) = 0;

    virtual void get_ultrasonic_range_callback(const Get_Ultrasonic_Range_Resp& resp) = 0;

    virtual void get_battery_voltage_callback(const Get_Battery_Voltage_Resp& resp) = 0;
};

}

#endif  // CROBOT_CONTROLLER_CALLBACK_H
