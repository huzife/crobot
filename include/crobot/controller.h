#ifndef CROBOT_CONTROLLER_H
#define CROBOT_CONTROLLER_H

#include "crobot/controller_callbacks.h"
#include "crobot/listener.h"
#include "crobot/swsr_queue.h"
#include "crobot/message/request.h"
#include "CSerialPort/SerialPort.h"
#include <thread>

namespace crobot {

class Controller {
private:
    itas109::CSerialPort sp_;
    crobot::Listener listener_;
    Controller_Callbacks& callbacks_;
    SWSR_Queue<uint8_t> data_queue_;

    bool thread_end_ = false;
    std::thread base_com_thread_;

public:
    Controller(Controller_Callbacks& cbs);
    ~Controller();

    void init(const char* port_name,
              itas109::BaudRate baudrate,
              itas109::Parity parity,
              itas109::DataBits databits,
              itas109::StopBits stopbits,
              itas109::FlowControl flow_control);
    bool open();

    // receive
    void receive_data(uint8_t* data, uint32_t len);
    void process_response(const Response& resp);
    void base_com_func();

    // send
    void send_request(const Request& req);

private:
    void write(const Request& req);
};

} // namespace crobot

#endif // CROBOT_CONTROLLER_H
