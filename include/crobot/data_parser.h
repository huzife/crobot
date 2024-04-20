#ifndef CROBOT_DATA_PARSER_H
#define CROBOT_DATA_PARSER_H

#include "crobot/message/response.h"
#include <cstdint>
#include <vector>

namespace crobot {

class Data_Parser {
private:
    enum class State {
        RX_HEADER_FE,
        RX_HEADER_EF,
        RX_LEN,
        RX_DATA
    };

public:
    Data_Parser(uint32_t buf_len = 128);

    bool parse(uint8_t data);

    Response get_response();

private:
    State state_;
    std::vector<uint8_t> buf_;
    uint32_t buf_len_;
    uint32_t data_len_;
};

} // namespace crobot

#endif // CROBOT_DATA_PARSER_H
