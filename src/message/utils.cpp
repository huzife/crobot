#include "crobot/message/utils.h"

namespace crobot {

void float_to_hex(float fval, std::vector<uint8_t>& hex, int offset) {
    Float_Hex fh;
    fh.fval = fval;
    for (int i = 0; i < 4; i++) {
        hex[i + offset] = fh.hex[3 - i];
    }
}

float hex_to_float(const std::vector<uint8_t>& hex, int offset) {
    Float_Hex fh;
    for (int i = 0; i < 4; i++) {
        fh.hex[3 - i] = hex[i + offset];
    }
    return fh.fval;
}

} // namespace crobot
