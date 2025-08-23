#include "cobs.hpp"

bool cobs_encode(std::span<const uint8_t> in, std::span<uint8_t> out, std::size_t& out_len) noexcept {
    const uint8_t* src = in.data(); const uint8_t* end = src + in.size();
    uint8_t* dst = out.data(); uint8_t* code_ptr = dst++; uint8_t code = 1;

    while (src < end) {
        if (*src == 0) { *code_ptr = code; code_ptr = dst++; code = 1; ++src; }
        else { *dst++ = *src++; if (++code == 0xFF) { *code_ptr = code; code_ptr = dst++; code = 1; } }
        if ((std::size_t)(dst - out.data()) >= out.size()) return false;
    }

    *code_ptr = code;
    out_len = (std::size_t)(dst - out.data());
    return true;
}