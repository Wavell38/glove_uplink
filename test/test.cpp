// main.cpp
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// === Réglages (modifie si besoin) ===
#ifndef UART_ID
#define UART_ID       uart0
#endif
#ifndef UART_BAUD
#define UART_BAUD     460800
#endif
#ifndef UART_TX_PIN
#define UART_TX_PIN   0   // GP0  (TX0 par défaut)
#endif
#ifndef UART_RX_PIN
#define UART_RX_PIN   1   // GP1  (RX0 par défaut)
#endif

int main() {
    // Pas d'attente USB
    stdio_init_all(); // ok même si on ne l'utilise pas

    // Init UART
    uart_init(UART_ID, UART_BAUD);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_fifo_enabled(UART_ID, true);

    // Mappage des GPIO vers la fonction UART
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    const uint8_t ping[] = { 'P','I','N','G','\n' };
    const uint8_t z = 0x00;
    int counter = 0;

    while (true) {
        uart_write_blocking(UART_ID, ping, sizeof(ping));
        if ((++counter % 10) == 0) {           // toutes les 10 lignes
            uart_write_blocking(UART_ID, &z, 1); // envoie 0x00 (test pratique)
        }
        sleep_ms(100);
    }
}
