#ifndef CROBOT_LISTENER_H
#define CROBOT_LISTENER_H

#include "CSerialPort/SerialPort.h"
#include <functional>

namespace crobot {


class Listener: public itas109::CSerialPortListener {
public:
    using Listener_Read_Callback = std::function<void(uint8_t*, uint32_t)>;

    static constexpr uint32_t READ_BUF_SIZE = 1024;

private:
    itas109::CSerialPort& sp_;
    Listener_Read_Callback read_callback_;
    uint8_t read_data_[READ_BUF_SIZE];

public:
    Listener(itas109::CSerialPort& sp, Listener_Read_Callback cb): sp_(sp), read_callback_(cb) {}

    void onReadEvent(const char* port_name, uint32_t read_buf_len) override;
};

} // namespace crobot

#endif // CROBOT_LISTENER_H
