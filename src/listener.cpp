#include "crobot/listener.h"

namespace crobot {

void Listener::onReadEvent(const char* port_name, uint32_t read_buf_len) {
    if (read_buf_len == 0) return;

    // 读数据
    int len = sp_.readData(read_data_, read_buf_len);
    if (len > 0)
        read_callback_(read_data_, len);
}

} // namespace crobot
