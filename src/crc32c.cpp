#include "crc32c.hpp"

uint32_t crc32c(std::span<const uint8_t> d) noexcept {
    uint32_t c = 0xFFFFFFFFu;
    for (uint8_t b : d) {
        c ^= b;
        for (int i=0;i<8;i++) c = (c >> 1) ^ (0x82F63B78u & (-(int)(c & 1)));
    }
    return ~c;
}