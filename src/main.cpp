/*#include <array>
#include <cstdio>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/timer.h"

#include "pins.hpp"
#include "proto.hpp"
#include "dev_i2c.hpp"
#include "mpu6050.hpp"
#include "icm20948.hpp"
#include "cobs.hpp"
#include "crc32c.hpp"
#include "log.hpp"
#include "scan.hpp"

static constexpr uint8_t  TCA_ADDR = 0x70;
static constexpr uint32_t BAUD     = 460800;
static constexpr int      PERIOD_US= 5000; // 200 Hz

struct Stats {
    uint32_t ok{0}, err{0}, loops{0};
    uint32_t max_loop_us{0};
    uint32_t last_log_ms{0};
};

// Mapping canal TCA -> (type, id logique)
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

// Nom court pour logs
static inline const char* short_name(SensorId id) {
    switch(id){
        case SensorId::Thumb:       return "Th";
        case SensorId::IndexProx:   return "IP";
        case SensorId::IndexDist:   return "ID";
        case SensorId::MiddleProx:  return "MP";
        case SensorId::MiddleDist:  return "MD";
        case SensorId::RingProx:    return "RP";
        case SensorId::RingDist:    return "RD";
        case SensorId::BackPalm:    return "BP";
        default: return "--";
    }
}

int main() {
    stdio_init_all();
   
     // Attendre que /dev/ttyACM* soit ouvert (max 3 s)
    absolute_time_t deadline = make_timeout_time_ms(3000);
    while (!stdio_usb_connected() && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        tight_loop_contents();
    }

    sleep_ms(200); // laisse au host le temps de s'attacher
    printf("USB ready\n"); fflush(stdout);
    LOGI("Boot OK (USB connected or timeout)");

    // I2C0 @ 400 kHz
    i2c_init(i2c0, 400000);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C0_SDA);
    gpio_pull_up(PIN_I2C0_SCL);

    // UART0 @ 460800
    uart_init(uart0, BAUD);
    gpio_set_function(PIN_UART0_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART0_RX, GPIO_FUNC_UART);

    ChDiag diag[8]{};
   
    [&]() {
        LOGI("Probe TCA @0x%02X", TCA_ADDR);

        for (int ch=0; ch<8; ++ch) {

            if (!tca_select(i2c0, TCA_ADDR, ch)) {
                LOGE(" ch%d: TCA select NACK", ch);
                diag[ch].present = false;
                diag[ch].last = ErrCode::NACK;
                continue;
            }
            sleep_us(5);

            uint8_t v=0;
            // MPU6050 WHO_AM_I
            bool is_mpu = (i2c_r(i2c0, 0x68, 0x75, &v, 1) >= 0) && (v == 0x68);
            // ICM-20948 WHO_AM_I (BANK0)
            bool is_icm = (i2c_r(i2c0, 0x68, 0x00, &v, 1) >= 0) && (v == 0xEA);

            if (is_mpu) {
                diag[ch].present = true; diag[ch].kind = DevKind::MPU; diag[ch].last = ErrCode::OK;
                LOGI(" ch%d: MPU6050 present", ch);
            } else if (is_icm) {
                diag[ch].present = true; diag[ch].kind = DevKind::ICM; diag[ch].last = ErrCode::OK;
                LOGI(" ch%d: ICM-20948 present", ch);
            } else {
                diag[ch].present = false; diag[ch].last = ErrCode::WHOAMI;
                LOGW(" ch%d: no known WHO_AM_I (mpu/icm)", ch);
            }
        }
    } ();

    for (int i=0;i<8;i++) {
        const auto& d = kDevices[i];
        if (!diag[d.tca_ch].present) continue;
        if (!tca_select(i2c0, TCA_ADDR, d.tca_ch)) { LOGE("Init: TCA sel fail ch%d", d.tca_ch); diag[d.tca_ch].last=ErrCode::NACK; continue; }
        sleep_us(200); // BOOT uniquement

        bool ok=false;
        if (diag[d.tca_ch].kind == DevKind::MPU) { Mpu6050 m{}; ok = m.init(i2c0); }
        else                                      { Icm20948 c{}; ok = c.init(i2c0); }

        if (ok) LOGI("Init OK ch%d (%s)", d.tca_ch, diag[d.tca_ch].kind==DevKind::MPU?"MPU":"ICM");
        else   { LOGE("Init FAIL ch%d (%s)", d.tca_ch, diag[d.tca_ch].kind==DevKind::MPU?"MPU":"ICM"); diag[d.tca_ch].last=ErrCode::OTHER; }
    }
    // Init capteurs via TCA
    /*for (auto& d : kDevices) {
        
        tca_select(i2c0, TCA_ADDR, d.tca_ch);
        sleep_us(200);

        bool ok=false;

        if (d.kind == DevKind::MPU) { Mpu6050 m{}; ok = m.init(i2c0); }
        else                        { Icm20948 c{}; ok = c.init(i2c0); }
        if (ok) LOGI("Init OK ch%d (%s)", d.tca_ch, d.kind==DevKind::MPU ? "MPU6050":"ICM-20948");
        else    LOGE("Init FAIL ch%d (%s)", d.tca_ch, d.kind==DevKind::MPU ? "MPU6050":"ICM-20948");
    }

    absolute_time_t next = delayed_by_us(get_absolute_time(), PERIOD_US);
    uint32_t seq = 0;
    Stats st;

    while (true) {

        // Prépare le batch
        PacketHeader hdr{};
        hdr.magic    = 0xA5;
        hdr.version  = 1;
        hdr.msg_type = 0;   // IMU_RAW_I16
        hdr.seq      = seq++;
        hdr.t_uC_us  = (uint32_t)to_us_since_boot(get_absolute_time());
        hdr.count    = 8;
        hdr.reserved = present_mask;   // bitmask de présence (TCA)

        uint32_t i2c_busy_us = 0;
        RawItemI16 items[8];
        uint8_t present_mask = 0;

        for (int i=0;i<8;i++) {
            const auto& d = kDevices[i];
            auto& cd = diag[d.tca_ch];

            if (cd.present) present_mask |= (1u << d.tca_ch);
            if (!cd.present) { cd.err++; cd.last = ErrCode::NACK; continue; }

            const uint32_t ti0 = (uint32_t)to_us_since_boot(get_absolute_time());

            if (!tca_select(i2c0, TCA_ADDR, d.tca_ch)) {
                cd.err++; cd.last = ErrCode::NACK;
                continue;
            }
            sleep_us(5);

            bool ok = false;
            int16_t acc[3]{}, gyr[3]{}, tmp{};
            if (cd.kind == DevKind::MPU) {
                Mpu6050 m{}; ok = m.read14(i2c0, acc, tmp, gyr);
            } else {
                Icm20948 c{}; ok = c.read_acc_gyro(i2c0, acc, gyr, tmp);
            }

            const uint32_t ti1 = (uint32_t)to_us_since_boot(get_absolute_time());
            i2c_busy_us += (ti1 - ti0);   // <<< additionne le temps I²C

            items[i].sensor_id = static_cast<uint8_t>(d.id);
            items[i].reserved  = 0;

            if (ok) {
                items[i].acc[0]=acc[0]; items[i].acc[1]=acc[1]; items[i].acc[2]=acc[2];
                items[i].gyr[0]=gyr[0]; items[i].gyr[1]=gyr[1]; items[i].gyr[2]=gyr[2];
                items[i].temp   = tmp;
                items[i].mag[0]=items[i].mag[1]=items[i].mag[2]=INT16_C(0x7FFF);
                st.ok++;  cd.ok++;        // <<< incrémente aussi le canal
                cd.last = ErrCode::OK;
            } else {
                items[i].acc[0]=items[i].acc[1]=items[i].acc[2]=0;
                items[i].gyr[0]=items[i].gyr[1]=items[i].gyr[2]=0;
                items[i].temp = 0;
                items[i].mag[0]=items[i].mag[1]=items[i].mag[2]=INT16_C(0x7FFF);
                st.err++; cd.err++;       // <<< incrémente aussi le canal
                cd.last = ErrCode::READ;
            }
        }

        hdr.reserved = present_mask;

        const uint32_t t1 = (uint32_t)to_us_since_boot(get_absolute_time());
        const uint32_t loop_us = t1 - t0;
        if (loop_us > st.max_loop_us) st.max_loop_us = loop_us;

            // 4) Encodage + (optionnel) envoi UART
        #if ENABLE_DATA_TX
            uint8_t frame[16 + 8*sizeof(RawItemI16) + 4];
            memcpy(frame, &hdr, sizeof(hdr));
            memcpy(frame + sizeof(hdr), items, sizeof(items));
            uint32_t crc = crc32c({frame, sizeof(frame)-4});
            memcpy(frame + sizeof(frame)-4, &crc, 4);

            uint8_t enc[512]; std::size_t enc_len=0;
            cobs_encode({frame, sizeof(frame)}, {enc, sizeof(enc)}, enc_len);
            uart_write_blocking(uart0, enc, enc_len);
            uint8_t z=0; uart_write_blocking(uart0, &z, 1);
        #endif

        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());  // <<< manquait
        // 5) Logging périodique (1 Hz par défaut)
        if (now_ms - st.last_log_ms >= LOG_EVERY_MS) {
            const float occ_loop = (loop_us / 5000.0f) * 100.0f;
            const float occ_i2c  = (i2c_busy_us / 5000.0f) * 100.0f;

            // Aggrégats
            uint32_t ok_tot=0, err_tot=0;
            for (int ch=0; ch<8; ++ch) { ok_tot += diag[ch].ok; err_tot += diag[ch].err; }

            LOGI("200Hz loop: loop=%.2f ms (%.0f%%), i2c=%.2f ms (%.0f%%), ok=%u, err=%u",
                loop_us/1000.0f, occ_loop, i2c_busy_us/1000.0f, occ_i2c, ok_tot, err_tot);

            // Ligne synthétique par canal : [ch][name][P/A][OK][ER][last]
            for (int i=0;i<8;i++) {
            const auto& d  = kDevices[i];
            const auto& cd = diag[d.tca_ch];
            const char* typ = (cd.kind==DevKind::MPU?"MPU":"ICM");
            const char* last =
                (cd.last==ErrCode::OK?"OK":
                cd.last==ErrCode::NACK?"NACK":
                cd.last==ErrCode::WHOAMI?"WHOAMI":
                cd.last==ErrCode::READ?"READ":"OTH");
            LOGI(" ch%d %-2s %-3s  pres=%d  OK=%4u  ER=%4u  last=%s",
                d.tca_ch, short_name(d.id), typ, cd.present?1:0, cd.ok, cd.err, last);
            }

            // reset la fenêtre 1 s (si tu veux des deltas par seconde)
            for (auto& cd : diag) { cd.ok = cd.err = 0; }
            st.max_loop_us = 0;
            st.last_log_ms = now_ms;
        }

        st.loops++;
        sleep_until(next);
        next = delayed_by_us(next, PERIOD_US);
    }
}*/

#include <array>
#include <cstdio>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/timer.h"

#include "pins.hpp"
#include "proto.hpp"
#include "dev_i2c.hpp"
#include "mpu6050.hpp"
#include "icm20948.hpp"
#include "cobs.hpp"
#include "crc32c.hpp"
#include "log.hpp"

static constexpr uint8_t  TCA_ADDR = 0x70;
static constexpr uint32_t BAUD     = 460800;
static constexpr int      PERIOD_US= 5000; // 200 Hz

struct Stats {
    uint32_t ok{0}, err{0}, loops{0};
    uint32_t max_loop_us{0};
    uint32_t last_log_ms{0};
};

// Mapping canal TCA -> (type, id logique)
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

// Nom court pour logs
static inline const char* short_name(SensorId id) {
    switch(id){
        case SensorId::Thumb:       return "Th";
        case SensorId::IndexProx:   return "IP";
        case SensorId::IndexDist:   return "ID";
        case SensorId::MiddleProx:  return "MP";
        case SensorId::MiddleDist:  return "MD";
        case SensorId::RingProx:    return "RP";
        case SensorId::RingDist:    return "RD";
        case SensorId::BackPalm:    return "BP";
        default: return "--";
    }
}

int main() {
    stdio_init_all();

    // Attendre que /dev/ttyACM* soit ouvert (max 3 s)
    absolute_time_t deadline = make_timeout_time_ms(3000);
    while (!stdio_usb_connected() && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        tight_loop_contents();
    }
    sleep_ms(200);
    LOGI("Boot OK (USB connected or timeout)");

    // I2C0 @ 400 kHz
    i2c_init(i2c0, 400000);
    gpio_set_function(PIN_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C0_SDA);
    gpio_pull_up(PIN_I2C0_SCL);

    // UART0 @ 460800
    uart_init(uart0, BAUD);
    gpio_set_function(PIN_UART0_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART0_RX, GPIO_FUNC_UART);

    // --- Inventaire au boot ---
    ChDiag diag[8]{};

    // Exécuter la lambda, ne pas l'assigner (sinon 'auto' déduit 'void')
    [&](){
        LOGI("Probe TCA @0x%02X", TCA_ADDR);
        for (int ch=0; ch<8; ++ch) {
            if (!tca_select(i2c0, TCA_ADDR, ch)) {
                LOGE(" ch%d: TCA select NACK", ch);
                diag[ch].present = false;
                diag[ch].last    = ErrCode::NACK;
                continue;
            }
            sleep_us(5);

            uint8_t v=0;
            bool is_mpu = (i2c_r(i2c0, 0x68, 0x75, &v, 1) >= 0) && (v == 0x68);
            bool is_icm = (i2c_r(i2c0, 0x68, 0x00, &v, 1) >= 0) && (v == 0xEA);

            if (is_mpu) {
                diag[ch].present = true; diag[ch].kind = DevKind::MPU; diag[ch].last = ErrCode::OK;
                LOGI(" ch%d: MPU6050 present", ch);
            } else if (is_icm) {
                diag[ch].present = true; diag[ch].kind = DevKind::ICM; diag[ch].last = ErrCode::OK;
                LOGI(" ch%d: ICM-20948 present", ch);
            } else {
                diag[ch].present = false; diag[ch].last = ErrCode::WHOAMI;
                LOGW(" ch%d: no known WHO_AM_I (mpu/icm)", ch);
            }
        }
    }(); // <-- appel immédiat

    // Init capteurs présents
    for (int i=0;i<8;i++) {
        const auto& d = kDevices[i];
        if (!diag[d.tca_ch].present) continue;
        if (!tca_select(i2c0, TCA_ADDR, d.tca_ch)) {
            LOGE("Init: TCA sel fail ch%d", d.tca_ch);
            diag[d.tca_ch].last=ErrCode::NACK;
            continue;
        }
        sleep_us(200); // BOOT uniquement

        bool ok=false;
        if (diag[d.tca_ch].kind == DevKind::MPU) { Mpu6050 m{}; ok = m.init(i2c0); }
        else                                      { Icm20948 c{}; ok = c.init(i2c0); }

        if (ok) LOGI("Init OK ch%d (%s)", d.tca_ch, diag[d.tca_ch].kind==DevKind::MPU?"MPU":"ICM");
        else   { LOGE("Init FAIL ch%d (%s)", d.tca_ch, diag[d.tca_ch].kind==DevKind::MPU?"MPU":"ICM"); diag[d.tca_ch].last=ErrCode::OTHER; }
    }

    absolute_time_t next = delayed_by_us(get_absolute_time(), PERIOD_US);
    uint32_t seq = 0;
    Stats st;

    while (true) {
        const uint32_t t0 = (uint32_t)to_us_since_boot(get_absolute_time());

        uint32_t i2c_busy_us = 0;
        uint8_t  present_mask = 0;
        RawItemI16 items[8];

        // Acquisition séquentielle
        for (int i=0;i<8;i++) {
            const auto& d = kDevices[i];
            auto& cd = diag[d.tca_ch];

            if (cd.present) present_mask |= (1u << d.tca_ch);
            if (!cd.present) { cd.err++; cd.last = ErrCode::NACK; continue; }

            const uint32_t ti0 = (uint32_t)to_us_since_boot(get_absolute_time());

            if (!tca_select(i2c0, TCA_ADDR, d.tca_ch)) {
                cd.err++; cd.last=ErrCode::NACK;
                continue;
            }

            sleep_us(5); // 0–10 µs suffisent en général

            bool ok = false;
            int16_t acc[3]{}, gyr[3]{}, tmp{};
            if (cd.kind == DevKind::MPU) {
                Mpu6050 m{}; ok = m.read14(i2c0, acc, tmp, gyr);
            } else {
                Icm20948 c{}; ok = c.read_acc_gyro(i2c0, acc, gyr, tmp);
            }

            const uint32_t ti1 = (uint32_t)to_us_since_boot(get_absolute_time());
            i2c_busy_us += (ti1 - ti0);

            items[i].sensor_id = static_cast<uint8_t>(d.id);
            items[i].reserved  = 0;

            if (ok) {
                items[i].acc[0]=acc[0]; items[i].acc[1]=acc[1]; items[i].acc[2]=acc[2];
                items[i].gyr[0]=gyr[0]; items[i].gyr[1]=gyr[1]; items[i].gyr[2]=gyr[2];
                items[i].temp   = tmp;
                items[i].mag[0]=items[i].mag[1]=items[i].mag[2]=INT16_C(0x7FFF);
                st.ok++;  cd.ok++;
                cd.last = ErrCode::OK;
            } else {
                items[i].acc[0]=items[i].acc[1]=items[i].acc[2]=0;
                items[i].gyr[0]=items[i].gyr[1]=items[i].gyr[2]=0;
                items[i].temp = 0;
                items[i].mag[0]=items[i].mag[1]=items[i].mag[2]=INT16_C(0x7FFF);
                st.err++; cd.err++;
                cd.last = ErrCode::READ;
            }
        }

        // Header explicite
        PacketHeader hdr{};
        hdr.magic    = 0xA5;
        hdr.version  = 1;
        hdr.msg_type = 0; // IMU_RAW_I16
        hdr.seq      = seq++;
        hdr.t_uC_us  = t0;
        hdr.count    = 8;
        hdr.reserved = present_mask; // bitmask présence TCA

        const uint32_t t1 = (uint32_t)to_us_since_boot(get_absolute_time());
        const uint32_t loop_us = t1 - t0;
        if (loop_us > st.max_loop_us) st.max_loop_us = loop_us;

        // (Option) envoi UART prototypé
    #if ENABLE_DATA_TX
        uint8_t frame[16 + 8*sizeof(RawItemI16) + 4];
        memcpy(frame, &hdr, sizeof(hdr));
        memcpy(frame + sizeof(hdr), items, sizeof(items));
        uint32_t crc = crc32c({frame, sizeof(frame)-4});
        memcpy(frame + sizeof(frame)-4, &crc, 4);

        uint8_t enc[512]; std::size_t enc_len=0;
        cobs_encode({frame, sizeof(frame)}, {enc, sizeof(enc)}, enc_len);
        uart_write_blocking(uart0, enc, enc_len);
        uint8_t z=0; uart_write_blocking(uart0, &z, 1);
    #endif

        // Logging périodique (1 Hz)
        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - st.last_log_ms >= LOG_EVERY_MS) {
            const float occ_loop = (loop_us / 5000.0f) * 100.0f;
            const float occ_i2c  = (i2c_busy_us / 5000.0f) * 100.0f;

            uint32_t ok_tot=0, err_tot=0;
            for (int ch=0; ch<8; ++ch) { ok_tot += diag[ch].ok; err_tot += diag[ch].err; }

            LOGI("200Hz loop: loop=%.2f ms (%.0f%%), i2c=%.2f ms (%.0f%%), ok=%u, err=%u",
                 loop_us/1000.0f, occ_loop, i2c_busy_us/1000.0f, occ_i2c, ok_tot, err_tot);

            for (int i=0;i<8;i++) {
                const auto& d  = kDevices[i];
                const auto& cd = diag[d.tca_ch];
                const char* typ = (cd.kind==DevKind::MPU?"MPU":"ICM");
                const char* last =
                    (cd.last==ErrCode::OK?"OK":
                     cd.last==ErrCode::NACK?"NACK":
                     cd.last==ErrCode::WHOAMI?"WHOAMI":
                     cd.last==ErrCode::READ?"READ":"OTH");
                LOGI(" ch%d %-2s %-3s  pres=%d  OK=%4u  ER=%4u  last=%s",
                     d.tca_ch, short_name(d.id), typ, cd.present?1:0, cd.ok, cd.err, last);
            }

            // fenêtre 1 s
            for (auto& cd : diag) { cd.ok = cd.err = 0; }
            st.max_loop_us = 0;
            st.last_log_ms = now_ms;
        }

        st.loops++;
        sleep_until(next);
        next = delayed_by_us(next, PERIOD_US);
    }
}
