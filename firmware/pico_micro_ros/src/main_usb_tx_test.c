#include <stdio.h>

#include "pico/stdlib.h"

int main(void) {
    stdio_init_all();
    while (true) {
        putchar_raw('T');
        sleep_ms(100);
    }
    return 0;
}
