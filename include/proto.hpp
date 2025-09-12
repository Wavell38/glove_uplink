#pragma once
#include <cstdint>

//reserved 
//bit0 : VALID     (1 = l’échantillon est valide)
//bit1 : PRESENT   (1 = device présent à l’instant T)
//bit3..2 : KIND   (00=MPU, 01=ICM, 10..11 réservés)
//bit7..4 : ERR    (0=OK, 1=NACK, 2=WHOAMI, 3=READ, 4=OTHER, sinon réservé)
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

struct RawItemI16_NoMag {
    uint8_t  sensor_id;
    uint8_t  reserved;
    int16_t  acc[3];
    int16_t  gyr[3];
    int16_t  temp;
} __attribute__((packed));

// item compact pour le supplément mag (8 octets)
struct MagSuppItem {
    uint8_t  sensor_id;
    uint8_t  pad;      // alignement
    int16_t  mag[3];
} __attribute__((packed));

enum class MsgType : uint8_t {
    IMU_RAW_I16        = 0, // (ancien) 22 o/item (acc+gyr+temp+mag)
    IMU_RAW_I16_NOMAG  = 1, // (nouveau) 16 o/item (acc+gyr+temp)
    IMU_MAG_SUPP       = 2, // (nouveau)  8 o/item (id + mag[3])
};

// IDs exacts (ceux que tu veux côté Pi)
enum class SensorId : uint8_t {
    Thumb, IndexProx, IndexDist, MiddleProx, MiddleDist, RingProx, RingDist, BackPalm
};

static_assert(sizeof(PacketHeader) == 14, "PacketHeader must be 14B");
static_assert(sizeof(RawItemI16)  == 22, "RawItemI16 must be 22B");
static_assert(sizeof(RawItemI16_NoMag) == 16, "RawItemI16_NoMag must be 16B");
static_assert(sizeof(MagSuppItem)         == 8,  "MagSuppItem must be 8B");
