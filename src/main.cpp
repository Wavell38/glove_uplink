// main.cpp — RP2040 uplink (mixte NoMag + supplément Mag)
// - UART_BAUD, UART_TX_PIN, UART_RX_PIN, PIN_I2C_SDA, PIN_I2C_SCL doivent être
//   définis via CMake (ou via pins.hpp si tu préfères).
// - Requiert proto.hpp (avec MsgType, RawItemI16_NoMag, MagSuppItem).

#include <array>
#include <cstdio>
#include <cstring>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "pico/version.h"

#include "proto.hpp"
#include "dev_i2c.hpp"   // tca_select, i2c_r, i2c_read_timeout_us, etc.
#include "mpu6050.hpp"
#include "icm20948.hpp"
#include "cobs.hpp"
#include "crc32c.hpp"
#include "log.hpp"

//========================
// Options générales
//========================
#ifndef ENABLE_DATA_TX
#define ENABLE_DATA_TX 1
#endif

#ifndef LOG_EVERY_MS
#define LOG_EVERY_MS 1000
#endif

// Active le supplément MAG (trame IMU_MAG_SUPP) si tu as une lecture mag opérationnelle
#ifndef ENABLE_MAG_SUPP
#define ENABLE_MAG_SUPP 0
#endif

#define DIAG_MODE       0
#define DIRECT_MODE     0
#define BYPASS_TCA_TEST 0

static constexpr uint8_t  DEFAULT_TCA_ADDR = 0x70;
static constexpr int      PERIOD_US       = 5000; // 200 Hz
static i2c_inst_t* const  I2C_BUS         = i2c1;

// Choix des capteurs (IDs logiques) qui publieront la mag quand ENABLE_MAG_SUPP=1
// (ex: BackPalm seulement)
static constexpr uint8_t MAG_SENSORS_MASK = (1u << static_cast<uint8_t>(SensorId::BackPalm));

//========================
// Encodage flags 'reserved'
// bit0: VALID (1=échantillon valide)
// bit1: PRESENT (1=capteur vu présent)
// bit2..3: kind (0=MPU, 1=ICM)
// bit4..7: err code
//========================
enum ItemKind : uint8_t { KIND_MPU=0, KIND_ICM=1 };
enum ItemErr  : uint8_t { ERR_OK=0, ERR_NACK=1, ERR_WHOAMI=2, ERR_READ=3, ERR_OTHER=4 };

static inline uint8_t mk_flags(bool present, ItemKind kind, ItemErr err, bool valid) {
    uint8_t f=0;
    if (valid)   f |= (1u<<0);
    if (present) f |= (1u<<1);
    f |= (uint8_t(kind)&0x3u) << 2;
    f |= (uint8_t(err) &0xFu) << 4;
    return f;
}

//========================
// Mapping canal TCA -> (type, id logique)
//========================
enum class DevKind { MPU, ICM };
struct DevMap { uint8_t tca_ch; DevKind kind; SensorId id; };

static const DevMap kDevices[8] = {
    {0, DevKind::MPU, SensorId::Thumb},
    {1, DevKind::MPU, SensorId::IndexProx},
    {2, DevKind::MPU, SensorId::IndexDist},
    {3, DevKind::MPU, SensorId::MiddleProx},
    {4, DevKind::MPU, SensorId::MiddleDist},
    {5, DevKind::MPU, SensorId::RingProx},
    {6, DevKind::MPU, SensorId::RingDist},
    {7, DevKind::ICM, SensorId::BackPalm}, // ICM-20948 (mag plus tard)
};

enum class ErrCode : uint8_t { OK=0, NACK=1, WHOAMI=2, READ=3, OTHER=4 };

struct ChDiag {
    bool     present{false};
    DevKind  kind{DevKind::MPU};
    uint32_t ok{0};
    uint32_t err{0};
    ErrCode  last{ErrCode::OK};
};

//========================
// Helpers DIAG
//========================
static int i2c_quick_ping(i2c_inst_t* i2c, uint8_t addr, uint32_t tout_us=3000) {
    uint8_t v=0;
    int r = i2c_read_timeout_us(i2c, addr, &v, 1, false, tout_us);
    return (r == 1) ? 1 : r;
}

static int find_tca_addr(i2c_inst_t* i2c) {
    LOGI("Scanning upstream for TCA on 0x70..0x77 (SDA=%d, SCL=%d)", PIN_I2C_SDA, PIN_I2C_SCL);
    for (uint8_t a=0x70; a<=0x77; ++a) {
        int r = i2c_quick_ping(i2c, a, 3000);
        if (r > 0) { LOGI("TCA candidate found @0x%02X", a); return a; }
    }
    LOGE("No TCA found on 0x70..0x77");
    return -1;
}

static int tca_read_ctrl(i2c_inst_t* i2c, uint8_t tca_addr, uint8_t &val) {
    int r = i2c_read_timeout_us(i2c, tca_addr, &val, 1, false, 3000);
    return (r==1) ? 0 : r;
}

static int mpu6050_whoami(i2c_inst_t* i2c, uint8_t addr) {
    uint8_t v=0; if (i2c_r(i2c, addr, 0x75, &v, 1) < 0) return -1; return v;
}
static int icm20948_whoami(i2c_inst_t* i2c, uint8_t addr) {
    uint8_t v=0; if (i2c_r(i2c, addr, 0x00, &v, 1) < 0) return -1; return v;
}

static void i2c_bus_recover(uint sda_gpio, uint scl_gpio) {
    gpio_init(sda_gpio); gpio_init(scl_gpio);
    gpio_set_dir(sda_gpio, GPIO_IN);
    gpio_set_dir(scl_gpio, GPIO_OUT);
    gpio_put(scl_gpio, 1); sleep_us(5);
    for (int i=0; i<9 && !gpio_get(sda_gpio); ++i) {
        gpio_put(scl_gpio, 0); sleep_us(5);
        gpio_put(scl_gpio, 1); sleep_us(5);
    }
    gpio_set_dir(sda_gpio, GPIO_OUT);
    gpio_put(sda_gpio, 0); sleep_us(5);
    gpio_put(scl_gpio, 1); sleep_us(5);
    gpio_put(sda_gpio, 1); sleep_us(5);
}

//========================
// main()
//========================
int main() {
    stdio_init_all();

    // I2C init
    i2c_bus_recover(PIN_I2C_SDA, PIN_I2C_SCL);
    i2c_init(I2C_BUS, 400000);
    #if (PICO_SDK_VERSION_MAJOR > 2) || (PICO_SDK_VERSION_MAJOR == 2 && PICO_SDK_VERSION_MINOR >= 3)
        i2c_set_timeout_us(I2C_BUS, 2000);
    #endif
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    // UART init (TX/RX pins fournis par CMake)
    uart_init(uart0, UART_BAUD);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart0, false, false);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Détection TCA / inventaire WHO_AM_I
    uint8_t tca_addr = DEFAULT_TCA_ADDR;
    int tca = find_tca_addr(I2C_BUS);
    if (tca < 0) {
        LOGE("TCA not detected upstream. Check wiring/RESET/VCC/GND.");
        while (true) { sleep_ms(1000); }
    }
    tca_addr = static_cast<uint8_t>(tca);
    uint8_t ctrl=0xFF;
    (void)tca_read_ctrl(I2C_BUS, tca_addr, ctrl);

    ChDiag diag[8]{};
    // Inventaire
    for (int ch=0; ch<8; ++ch) {
        if (!tca_select(I2C_BUS, tca_addr, ch)) {
            diag[ch].present=false; diag[ch].last=ErrCode::NACK; continue;
        }
        sleep_us(100);
        uint8_t v=0; bool is_mpu=false, is_icm=false;
        if (i2c_r(I2C_BUS, 0x68, 0x75, &v, 1) >= 0 && v == 0x68) is_mpu=true;
        else if (i2c_r(I2C_BUS, 0x69, 0x75, &v, 1) >= 0 && v == 0x68) is_mpu=true;
        if (!is_mpu) {
            if (i2c_r(I2C_BUS, 0x68, 0x00, &v, 1) >= 0 && v == 0xEA) is_icm=true;
        }
        if (is_mpu) { diag[ch].present=true; diag[ch].kind=DevKind::MPU; diag[ch].last=ErrCode::OK; }
        else if (is_icm) { diag[ch].present=true; diag[ch].kind=DevKind::ICM; diag[ch].last=ErrCode::OK; }
        else { diag[ch].present=false; diag[ch].last=ErrCode::WHOAMI; }
    }

    // Init capteurs présents
    for (int i=0;i<8;i++) {
        const auto& d = kDevices[i];
        if (!diag[d.tca_ch].present) continue;
        if (!tca_select(I2C_BUS, tca_addr, d.tca_ch)) {
            diag[d.tca_ch].last=ErrCode::NACK; continue;
        }
        sleep_us(200);
        bool ok=false;
        if (diag[d.tca_ch].kind == DevKind::MPU) { Mpu6050 m{}; ok = m.init(I2C_BUS); }
        else                                      { Icm20948 c{}; ok = c.init(I2C_BUS); }
        if (!ok) diag[d.tca_ch].last=ErrCode::OTHER;
    }

    absolute_time_t next = delayed_by_us(get_absolute_time(), PERIOD_US);
    uint32_t seq = 0;

    while (true) {
        const uint32_t t0 = (uint32_t)to_us_since_boot(get_absolute_time());

        // === buffers de sortie ===
        RawItemI16_NoMag items[8];      // trame de base: toujours 8 items NoMag
        MagSuppItem      mag_items[8];  // supplément mag (si activé et lu)
        int              mag_count = 0;
        uint8_t          present_mask = 0;

        // Acquisition séquentielle
        for (int i=0;i<8;i++) {
            const auto& d  = kDevices[i];
            auto& cd       = diag[d.tca_ch];
            auto& out      = items[i];

            // ID logique toujours rempli
            out.sensor_id = static_cast<uint8_t>(d.id);
            out.reserved  = 0;

            // Valeurs par défaut
            out.acc[0]=out.acc[1]=out.acc[2]=0;
            out.gyr[0]=out.gyr[1]=out.gyr[2]=0;
            out.temp = 0;

            ItemKind kind = (cd.kind==DevKind::MPU) ? KIND_MPU : KIND_ICM;

            if (cd.present) present_mask |= (1u << d.tca_ch);
            if (!cd.present) {
                cd.err++; cd.last = ErrCode::NACK;
                out.reserved = mk_flags(false, kind, ERR_NACK, false);
                continue;
            }

            if (!tca_select(I2C_BUS, tca_addr, d.tca_ch)) {
                cd.err++; cd.last=ErrCode::NACK;
                out.reserved = mk_flags(true, kind, ERR_NACK, false);
                continue;
            }

            sleep_us(5);

            bool ok = false;
            int16_t acc[3]{}, gyr[3]{}, tmp{};
            if (cd.kind == DevKind::MPU) {
                Mpu6050 m{}; ok = m.read14(I2C_BUS, acc, tmp, gyr);
            } else {
                Icm20948 c{}; ok = c.read_acc_gyro(I2C_BUS, acc, gyr, tmp);
            }

            if (ok) {
                out.acc[0]=acc[0]; out.acc[1]=acc[1]; out.acc[2]=acc[2];
                out.gyr[0]=gyr[0]; out.gyr[1]=gyr[1]; out.gyr[2]=gyr[2];
                out.temp = tmp;
                cd.ok++; cd.last=ErrCode::OK;
                out.reserved = mk_flags(true, kind, ERR_OK, true);

#if ENABLE_MAG_SUPP
                // Publier la mag uniquement pour certains IDs logiques ET si on sait la lire
                if ((MAG_SENSORS_MASK & (1u << static_cast<uint8_t>(d.id))) &&
                    cd.kind == DevKind::ICM && mag_count < 8)
                {
                    // TODO: Remplace ce bloc par ta vraie lecture mag (AK09916 via ICM)
                    // int16_t mx,my,mz;
                    // bool okm = c.read_mag(I2C_BUS, mx, my, mz);
                    bool okm = false; // <-- tant que non implémenté
                    if (okm) {
                        mag_items[mag_count].sensor_id = static_cast<uint8_t>(d.id);
                        mag_items[mag_count].pad       = 0;
                        // mag_items[mag_count].mag[0]=mx;
                        // mag_items[mag_count].mag[1]=my;
                        // mag_items[mag_count].mag[2]=mz;
                        mag_count++;
                    }
                }
#endif
            } else {
                cd.err++; cd.last=ErrCode::READ;
                out.reserved = mk_flags(true, kind, ERR_READ, false);
            }
        }

        // ---- Trame de base: NoMag ----
        PacketHeader hdr{};
        hdr.magic    = 0xA5;
        hdr.version  = 1;
        hdr.msg_type = static_cast<uint8_t>(MsgType::IMU_RAW_I16_NOMAG);
        hdr.seq      = seq++;
        hdr.t_uC_us  = t0;
        hdr.count    = 8;
        hdr.reserved = present_mask;

#if ENABLE_DATA_TX
        {
            uint8_t frame[sizeof(PacketHeader) + sizeof(items) + 4];
            memcpy(frame, &hdr, sizeof(hdr));
            memcpy(frame + sizeof(hdr), items, sizeof(items));
            uint32_t crc = crc32c({frame, sizeof(frame)-4});
            memcpy(frame + sizeof(frame)-4, &crc, 4);

            uint8_t enc[512]; std::size_t enc_len=0;
            cobs_encode({frame, sizeof(frame)}, {enc, sizeof(enc)}, enc_len);
            uart_write_blocking(uart0, enc, enc_len);
            uint8_t z=0; uart_write_blocking(uart0, &z, 1);
        }
#endif

        // ---- Supplément MAG (optionnel) ----
#if ENABLE_MAG_SUPP
        if (mag_count > 0) {
            PacketHeader mh{};
            mh.magic    = 0xA5;
            mh.version  = 1;
            mh.msg_type = static_cast<uint8_t>(MsgType::IMU_MAG_SUPP);
            mh.seq      = hdr.seq;      // même séquence
            mh.t_uC_us  = hdr.t_uC_us;  // timestamp identique
            mh.count    = static_cast<uint16_t>(mag_count);
            mh.reserved = 0;

            const size_t payload = mag_count * sizeof(MagSuppItem);
            uint8_t frame2[sizeof(PacketHeader) + 8*sizeof(MagSuppItem) + 4];
            memcpy(frame2, &mh, sizeof(mh));
            memcpy(frame2 + sizeof(mh), mag_items, payload);
            uint32_t crc2 = crc32c({frame2, sizeof(mh)+payload});
            memcpy(frame2 + sizeof(mh) + payload, &crc2, 4);

            uint8_t enc2[256]; std::size_t enc2_len=0;
            cobs_encode({frame2, sizeof(mh)+payload+4}, {enc2, sizeof(enc2)}, enc2_len);
            uart_write_blocking(uart0, enc2, enc2_len);
            uint8_t z2=0; uart_write_blocking(uart0, &z2, 1);
        }
#endif

        // cadence fixe 200 Hz
        sleep_until(next);
        next = delayed_by_us(next, PERIOD_US);
    }
}
