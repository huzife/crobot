#include "crobot/data_parser.h"

namespace crobot {

Data_Parser::Data_Parser(uint32_t buf_len)
    : state_(State::RX_HEADER_FE),
      buf_(buf_len),
      buf_len_(buf_len),
      data_len_(0) {
    buf_[0] = 0xFE;
    buf_[1] = 0xEF;
}

bool Data_Parser::parse(uint8_t data) {
    switch (state_) {
        case State::RX_HEADER_FE:
            if (data == 0xFE)
                state_ = State::RX_HEADER_EF;
            break;
        case State::RX_HEADER_EF:
            if (data == 0xEF)
                state_ = State::RX_LEN;
            else if (data != 0xFE)
                state_ = State::RX_HEADER_FE;
            break;
        case State::RX_LEN:
            if (data + 3 > buf_len_) {
                state_ = State::RX_HEADER_FE;
            } else {
                state_ = State::RX_DATA;
                data_len_ = 0;
                buf_[2] = data;
            }
            break;
        case State::RX_DATA:
            buf_[3 + data_len_] = data;
            if (++(data_len_) == buf_[2]) {
                state_ = State::RX_HEADER_FE;
                return true;
            }
            break;
    }

    return false;
}

Response Data_Parser::get_response() {
    return Response{buf_, static_cast<uint32_t>(buf_[2] + 3)};
}

} // namespace crobot
