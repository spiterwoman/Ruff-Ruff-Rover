#include <stdint.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

int main(void) {
    uint8_t marker = 'S';
    uint8_t err = 0;

    stdio_init_all();
    (void)pico_serial_transport_open(NULL);

    while (true) {
        (void)pico_serial_transport_write(NULL, &marker, 1, &err);
        sleep_ms(100);
    }

    return 0;
}
