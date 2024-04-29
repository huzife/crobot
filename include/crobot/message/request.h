#ifndef CROBOT_MESSAGE_REQUEST_H
#define CROBOT_MESSAGE_REQUEST_H

#include "crobot/robot_base/robot_base.h"
#include <cstdint>
#include <vector>

namespace crobot {

class Request {
public:
    Request() = default;
    virtual ~Request() = default;

    virtual std::vector<uint8_t> data() const = 0;
};

class Set_PID_Interval_Req: public Request {
public:
    Set_PID_Interval_Req(uint16_t pid_interval);
    ~Set_PID_Interval_Req() = default;

    std::vector<uint8_t> data() const override;

private:
    uint16_t pid_interval_;
};

class Set_Count_Per_Rev_Req: public Request {
public:
    Set_Count_Per_Rev_Req(uint16_t cpr);
    ~Set_Count_Per_Rev_Req() = default;

    std::vector<uint8_t> data() const override;

private:
    uint16_t cpr_;
};

class Set_Robot_Base_Req: public Request {
public:
    Set_Robot_Base_Req() = default;
    ~Set_Robot_Base_Req() = default;
};

class Set_Robot_Base_2WD_Req: public Set_Robot_Base_Req {
public:
    Set_Robot_Base_2WD_Req(const Robot_Base_2WD_Param& param);
    ~Set_Robot_Base_2WD_Req() = default;

    std::vector<uint8_t> data() const override;

private:
    Robot_Base_2WD_Param param_;
};

class Set_Robot_Base_3WO_Req: public Set_Robot_Base_Req {
public:
    Set_Robot_Base_3WO_Req(const Robot_Base_3WO_Param& param);
    ~Set_Robot_Base_3WO_Req() = default;

    std::vector<uint8_t> data() const override;

private:
    Robot_Base_3WO_Param param_;
};

class Set_Robot_Base_4WD_Req: public Set_Robot_Base_Req {
public:
    Set_Robot_Base_4WD_Req(const Robot_Base_4WD_Param& param);
    ~Set_Robot_Base_4WD_Req() = default;

    std::vector<uint8_t> data() const override;

private:
    Robot_Base_4WD_Param param_;
};

class Set_Correction_Factor_Req: public Request {
public:
    Set_Correction_Factor_Req(float linear, float angular);
    ~Set_Correction_Factor_Req() = default;

    std::vector<uint8_t> data() const override;

private:
    float linear_;
    float angular_;
};

class Set_Velocity_Req: public Request {
public:
    Set_Velocity_Req(float linear_x, float linear_y, float angular_z);
    ~Set_Velocity_Req() = default;

    std::vector<uint8_t> data() const override;

private:
    float linear_x_;
    float linear_y_;
    float angular_z_;
};

class Reset_Odometry_Req: public Request {
public:
    std::vector<uint8_t> data() const override;
};

class Get_Odometry_Req: public Request {
public:
    std::vector<uint8_t> data() const override;
};

class Get_IMU_Temperature_Req: public Request {
public:
    std::vector<uint8_t> data() const override;
};

class Get_IMU_Data_Req: public Request {
public:
    std::vector<uint8_t> data() const override;
};

class Get_Ultrasonic_Range_Req: public Request {
public:
    std::vector<uint8_t> data() const override;
};

class Get_Battery_Voltage_Req: public Request {
    std::vector<uint8_t> data() const override;
};

} // namespace crobot

#endif // CROBOT_MESSAGE_REQUEST_H
