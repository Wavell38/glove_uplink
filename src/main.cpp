#include <array>
#include <cstdio>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "pico/version.h"

#include "pins.hpp"
#include "proto.hpp"
#include "dev_i2c.hpp"   // tca_select, i2c_r/w*_timeout_us
#include "mpu6050.hpp"
#include "icm20948.hpp"
#include "cobs.hpp"
#include "crc32c.hpp"
#include "log.hpp"

//========================
// Config générale
//========================
#define DIAG_MODE    1   // 0 ou 1 selon que tu veux le bloc diagnostic au boot
#define DIRECT_MODE  0   // 1 = pas d'inventaire WHO_AM_I; on force 'present' via un masque

#define BYPASS_TCA_TEST  0  // mets à 1 le temps de tester

#define ENABLED_CHANNEL_MASK  0b01111111

static constexpr uint8_t  DEFAULT_TCA_ADDR = 0x70;
static constexpr uint32_t BAUD     = 460800;
static constexpr int      PERIOD_US= 5000; // 200 Hz

static i2c_inst_t* const I2C_BUS = i2c1;

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

//========================
// Helpers DIAG (static dans ce fichier)
//========================

static int i2c_quick_ping(i2c_inst_t* i2c, uint8_t addr, uint32_t tout_us=3000) {
    uint8_t v=0;
    // Vrai test: une lecture 1 octet. Réussite uniquement si r==1 (ACK reçu).
    int r = i2c_read_timeout_us(i2c, addr, &v, 1, false, tout_us);
    return (r == 1) ? 1 : r; // sinon NACK/TIMEOUT -> échec
}

static void probe_direct_mpu(i2c_inst_t* i2c) {
    // Essaye WHO_AM_I du MPU6050 @0x68 (sans TCA)
    uint8_t v=0; 
    int r = i2c_r(i2c, 0x68, 0x75, &v, 1);
    if (r >= 0) LOGI("Direct WHO_AM_I 0x68 -> 0x%02X (r=%d)", v, r);
    else        LOGE("Direct WHO_AM_I 0x68 -> ERROR (r=%d)", r);
}

static int find_tca_addr(i2c_inst_t* i2c) {
    LOGI("Scanning upstream for TCA on 0x70..0x77 (SDA=%d, SCL=%d)", PIN_I2C_SDA, PIN_I2C_SCL);
    for (uint8_t a=0x70; a<=0x77; ++a) {
        int r = i2c_quick_ping(i2c, a, 3000);
        if (r > 0) { LOGI("TCA candidate found @0x%02X", a); return a; }
        else       { LOGW("  no reply @0x%02X (r=%d)", a, r); }
    }
    LOGE("No TCA found on 0x70..0x77");
    return -1;
}

static int tca_read_ctrl(i2c_inst_t* i2c, uint8_t tca_addr, uint8_t &val) {
    // Le TCA9548A renvoie l’état de son registre de sélection en lecture "directe"
    int r = i2c_read_timeout_us(i2c, tca_addr, &val, 1, false, 3000);
    return (r==1) ? 0 : r;
}

static int mpu6050_whoami(i2c_inst_t* i2c, uint8_t addr) {
    uint8_t v=0; if (i2c_r(i2c, addr, 0x75, &v, 1) < 0) return -1; return v;
}
static int icm20948_whoami(i2c_inst_t* i2c, uint8_t addr) {
    uint8_t v=0; if (i2c_r(i2c, addr, 0x00, &v, 1) < 0) return -1; return v;
}

static void scan_downstream_by_channel(i2c_inst_t* i2c, uint8_t tca_addr) {
    LOGI("Scanning TCA9548A at 0x%02X ...", tca_addr);
    for (int ch=0; ch<8; ++ch) {
        if (!tca_select(i2c, tca_addr, ch)) {
            LOGE(" ch%d: select FAILED (addr=0x%02X)", ch, tca_addr);
            continue;
        }
        sleep_us(200);

        // Ping large
        int found = 0;
        char line[256]; int ofs = 0;
        ofs += snprintf(line+ofs, sizeof(line)-ofs, " ch%d devices:", ch);
        for (uint8_t a=0x03; a<=0x77; ++a) {
            if (a == tca_addr) continue;
            int r = i2c_quick_ping(i2c, a, 2000);
            if (r >= 0) { ofs += snprintf(line+ofs, sizeof(line)-ofs, " 0x%02X", a); found++; }
        }
        LOGI("%s (n=%d)", line, found);

        // WHO_AM_I ciblés (adresses usuelles)
        int wm68 = mpu6050_whoami(i2c, 0x68);
        int wm69 = mpu6050_whoami(i2c, 0x69);
        int wi68 = icm20948_whoami(i2c, 0x68);

        if (wm68 == 0x68) LOGI(" ch%d: MPU6050 @0x68 (WHO=0x68)", ch);
        if (wm69 == 0x68) LOGI(" ch%d: MPU6050 @0x69 (WHO=0x68)", ch);
        if (wi68 == 0xEA) LOGI(" ch%d: ICM-20948 @0x68 (WHO=0xEA)", ch);

        if (wm68 != 0x68 && wm69 != 0x68 && wi68 != 0xEA && found==0)
            LOGW(" ch%d: no known WHO_AM_I (mpu@68=%d, mpu@69=%d, icm@68=%d)", ch, wm68, wm69, wi68);
    }
}

static void i2c_bus_recover(uint sda_gpio, uint scl_gpio) {
    // Relâche I2C et bit-bang 9 pulses SCL pour libérer l’esclave
    gpio_init(sda_gpio); gpio_init(scl_gpio);
    gpio_set_dir(sda_gpio, GPIO_IN);
    gpio_set_dir(scl_gpio, GPIO_OUT);
    gpio_put(scl_gpio, 1); sleep_us(5);

    // si SDA bas, on pulse SCL 9x
    for (int i=0; i<9 && !gpio_get(sda_gpio); ++i) {
        gpio_put(scl_gpio, 0); sleep_us(5);
        gpio_put(scl_gpio, 1); sleep_us(5);
    }
    // STOP manuel: SDA monte quand SCL est haut (si possible)
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

    //pour attendre que je me connect usb pour debug
    sleep_ms(10000);

    // Attendre que /dev/ttyACM* soit ouvert (max 3 s)
    absolute_time_t deadline = make_timeout_time_ms(3000);
    while (!stdio_usb_connected() && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        tight_loop_contents();
    }
    sleep_ms(200);
    LOGI("Boot OK (USB connected or timeout)");

    i2c_bus_recover(PIN_I2C_SDA, PIN_I2C_SCL);
    // I2C_BUS @ 400 kHz pour diag (remontera à 400 kHz après validation si souhaité)
    i2c_init(I2C_BUS, 400000);
    #if (PICO_SDK_VERSION_MAJOR > 2) || (PICO_SDK_VERSION_MAJOR == 2 && PICO_SDK_VERSION_MINOR >= 3)
    // Disponible à partir du SDK 2.3+
        i2c_set_timeout_us(I2C_BUS, 2000);
    #endif
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    // UART0 @ 460800 (pour data COBS éventuelle)
    uart_init(uart0, BAUD);
    gpio_set_function(PIN_UART0_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART0_RX, GPIO_FUNC_UART);

#if BYPASS_TCA_TEST
    LOGW("BYPASS_TCA_TEST: lecture directe @0x68 (aucun TCA)");
    probe_direct_mpu(I2C_BUS);
#endif

    //========================
    // DIAG (juste après init I2C)
    //========================
    uint8_t tca_addr = DEFAULT_TCA_ADDR;
#if DIAG_MODE
    {
        LOGI("Upstream sweep 0x03..0x77:");
        int count=0;
        for (uint8_t a=0x03; a<=0x77; ++a) {
            int r = i2c_quick_ping(I2C_BUS, a, 2000);
            if (r > 0) { LOGI("  seen @0x%02X", a); ++count; }
        }
        LOGI("Upstream sweep done, %d device(s) seen.", count);

        int tca = find_tca_addr(I2C_BUS);
        if (tca < 0) {
            LOGE("TCA not detected upstream. Vérifie: /RESET tiré haut, VCC 3V3, GND commun, pins SDA/SCL corrects.");
            // On s'arrête ici pour éviter du spam
            while (true) { sleep_ms(1000); }
        }
        tca_addr = static_cast<uint8_t>(tca);
        uint8_t ctrl=0xFF;
        int rc = tca_read_ctrl(I2C_BUS, tca_addr, ctrl);
        LOGI("TCA=0x%02X ctrl-reg=0x%02X (rc=%d)", tca_addr, ctrl, rc);
        scan_downstream_by_channel(I2C_BUS, tca_addr);
        LOGI("DIAG terminé, poursuite du boot avec TCA=0x%02X", tca_addr);
    }
#endif

    ChDiag diag[8]{};

#if DIRECT_MODE
    // Pas d'inventaire WHO_AM_I : on déclare "présents" les canaux du masque.
    // Le 'kind' est pris de kDevices pour que l'init sache MPU vs ICM.
    LOGI("DirectMod: skipping WHO_AM_I inventory; using mask=0x%02X", (unsigned)ENABLED_CHANNEL_MASK);
    for (int ch=0; ch<8; ++ch) {
        bool en = (ENABLED_CHANNEL_MASK >> ch) & 0x1;
        diag[ch].present = en;
        // Déduire le kind à partir de kDevices (mapping TCA->(type,id))
        // On parcourt kDevices pour trouver l'entrée correspondant à ce canal.
        for (int i=0; i<8; ++i) {
            if (kDevices[i].tca_ch == ch) {
                diag[ch].kind = (kDevices[i].kind == DevKind::MPU) ? DevKind::MPU : DevKind::ICM;
                break;
            }
        }
        diag[ch].last = en ? ErrCode::OK : ErrCode::NACK;
    }
#else
    // Inventaire WHO_AM_I « classique »
    [&](){
        LOGI("Boot inventory via TCA @0x%02X", tca_addr);
        for (int ch=0; ch<8; ++ch) {
            if (!tca_select(I2C_BUS, tca_addr, ch)) {
                LOGE(" ch%d: TCA select NACK", ch);
                diag[ch].present = false;
                diag[ch].last    = ErrCode::NACK;
                continue;
            }
            sleep_us(100);

            // Tente MPU @0x68 et 0x69, puis ICM @0x68
            uint8_t v=0; bool is_mpu=false, is_icm=false;

            if (i2c_r(I2C_BUS, 0x68, 0x75, &v, 1) >= 0 && v == 0x68) { is_mpu=true; }
            else if (i2c_r(I2C_BUS, 0x69, 0x75, &v, 1) >= 0 && v == 0x68) { is_mpu=true; }

            if (!is_mpu) {
                if (i2c_r(I2C_BUS, 0x68, 0x00, &v, 1) >= 0 && v == 0xEA) { is_icm=true; }
            }

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
#endif

    // Init capteurs présents
    for (int i=0;i<8;i++) {
        const auto& d = kDevices[i];
        if (!diag[d.tca_ch].present) continue;
        if (!tca_select(I2C_BUS, tca_addr, d.tca_ch)) {
            LOGE("Init: TCA sel fail ch%d", d.tca_ch);
            diag[d.tca_ch].last=ErrCode::NACK;
            continue;
        }
        sleep_us(200); // BOOT uniquement

        bool ok=false;
        if (diag[d.tca_ch].kind == DevKind::MPU) { Mpu6050 m{}; ok = m.init(I2C_BUS); }
        else                                      { Icm20948 c{}; ok = c.init(I2C_BUS); }

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

            if (!tca_select(I2C_BUS, tca_addr, d.tca_ch)) {
                cd.err++; cd.last=ErrCode::NACK;
                continue;
            }

            sleep_us(5); // 0–10 µs suffisent en général

            bool ok = false;
            int16_t acc[3]{}, gyr[3]{}, tmp{};
            if (cd.kind == DevKind::MPU) {
                Mpu6050 m{}; ok = m.read14(I2C_BUS, acc, tmp, gyr);
            } else {
                Icm20948 c{}; ok = c.read_acc_gyro(I2C_BUS, acc, gyr, tmp);
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
                     (cd.last==ErrCode::NACK?"NACK":
                      (cd.last==ErrCode::WHOAMI?"WHOAMI":
                       (cd.last==ErrCode::READ?"READ":"OTH"))));
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
