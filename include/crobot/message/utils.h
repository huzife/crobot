#ifndef CROBOT_MESSAGE_UTILS_H
#define CROBOT_MESSAGE_UTILS_H

#include <cstdint>
#include <vector>

namespace crobot {

union Float_Hex {
    uint8_t hex[4];
    float fval;
};

void float_to_hex(float fval, std::vector<uint8_t>& hex, int offset);
float hex_to_float(const std::vector<uint8_t>& hex, int offset);

}

#endif  // CROBOT_MESSAGE_UTILS_H
