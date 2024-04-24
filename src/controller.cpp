#include "crobot/controller.h"
#include "crobot/data_parser.h"
#include "crobot/message/response.h"
#include <iostream>

namespace crobot {
Controller::Controller(Controller_Callbacks& cbs)
    : listener_(sp_, std::bind(&Controller::receive_data, this,
                             std::placeholders::_1,
                             std::placeholders::_2)),
      callbacks_(cbs),
      data_queue_(1024) {}

Controller::~Controller() {
    if (sp_.isOpen()) {
        std::cout << "close serial" << std::endl;
        sp_.close();
    }

    thread_end_ = true;
    base_com_thread_.join();
}

void Controller::init(const char* port_name,
                      itas109::BaudRate baudrate,
                      itas109::Parity parity,
                      itas109::DataBits databits,
                      itas109::StopBits stopbits,
                      itas109::FlowControl flow_control) {
    sp_.init(port_name, baudrate, parity, databits, stopbits, flow_control, Listener::READ_BUF_SIZE);
    sp_.setReadIntervalTimeout(0);
}

bool Controller::open() {
    if (sp_.open()) {
        std::cout << "Serial opened: " << sp_.getPortName() << std::endl;
    } else {
        std::cerr << "Failed to open serial: " << sp_.getLastErrorMsg() << std::endl;
        return false;
    }

    if (sp_.connectReadEvent(&listener_)) {
        std::cerr << "Failed to connect to read event" << std::endl;
        return false;
    }
    base_com_thread_ = std::thread{std::bind(&Controller::base_com_func, this)};

    return true;
}

void Controller::write(const Request& req) {
    auto data = req.data();
    if (sp_.writeData(data.data(), data.size()) == -1)
        std::cerr << "Failed to send data: " << sp_.getLastErrorMsg() << std::endl;
}

void Controller::receive_data(uint8_t* data, uint32_t len) {
    for (int i = 0; i < len; ++i) {
        data_queue_.push(data[i]);
    }
}

void Controller::process_response(const Response& resp) {
    switch (resp.type()) {
    case Message_Type::SET_VELOCITY:
        callbacks_.set_velocity_callback();
        break;
    case Message_Type::GET_ODOMETRY:
        callbacks_.get_odometry_callback(resp.get_odometry_resp());
        break;
    case Message_Type::GET_IMU_TEMPERATURE:
        callbacks_.get_imu_temperature_callback(resp.get_imu_temperature_resp());
        break;
    case Message_Type::GET_IMU_DATA:
        callbacks_.get_imu_data_callback(resp.get_imu_resp());
        break;
    case Message_Type::GET_ULTRASONIC_RANGE:
        callbacks_.get_ultrasonic_range_callback(resp.get_ultrasonic_range_resp());
        break;
    case Message_Type::GET_BATTERY_VOLTAGE:
        callbacks_.get_battery_voltage_callback(resp.get_battery_voltage_resp());
        break;
    default:
        break;
    }
}

void Controller::base_com_func() {
    Data_Parser parser;
    while (!thread_end_) {
        uint8_t data;
        if (!data_queue_.pop(data)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (parser.parse(data))
            process_response(parser.get_response());
    }
}


void Controller::send_request(const Request& req) {
    write(req);
}

} // namespace crobot
