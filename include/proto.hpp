#pragma once
#include <cstdint>

struct PacketHeader {
    uint8_t  magic{0xA5};
    uint8_t  version{1};
    uint8_t  msg_type{0}; // 0=IMU_RAW_I16
    uint8_t  reserved{0};
    uint32_t seq{0};
    uint32_t t_uC_us{0};
    uint16_t count{0};
} __attribute__((packed));

struct RawItemI16 {
    uint8_t  sensor_id;
    uint8_t  reserved;
    int16_t  acc[3];
    int16_t  gyr[3];
    int16_t  temp;
    int16_t  mag[3]; // 0x7FFF si absent
} __attribute__((packed));

// IDs exacts (ceux que tu veux côté Pi)
enum class SensorId : uint8_t {
    Thumb, IndexProx, IndexDist, MiddleProx, MiddleDist, RingProx, RingDist, BackPalm
};
