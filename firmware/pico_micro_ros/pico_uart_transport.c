#include <stdio.h>
#include <sys/types.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

static bool g_stdio_initialized = false;

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    (void)unused;
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool pico_serial_transport_open(struct uxrCustomTransport * transport)
{
    (void)transport;
    if (!g_stdio_initialized) {
        g_stdio_initialized = stdio_init_all();
    }
    return g_stdio_initialized;
}

bool pico_serial_transport_close(struct uxrCustomTransport * transport)
{
    (void)transport;
    return true;
}

size_t pico_serial_transport_write(
    struct uxrCustomTransport * transport,
    const uint8_t * buf,
    size_t len,
    uint8_t * errcode)
{
    (void)transport;

    for (size_t i = 0; i < len; ++i) {
        if (buf[i] != (uint8_t)putchar((int)buf[i])) {
            if (errcode != NULL) {
                *errcode = 1;
            }
            return i;
        }
    }

    if (errcode != NULL) {
        *errcode = 0;
    }
    return len;
}

size_t pico_serial_transport_read(
    struct uxrCustomTransport * transport,
    uint8_t * buf,
    size_t len,
    int timeout,
    uint8_t * errcode)
{
    (void)transport;
    uint64_t start_time_us = time_us_64();

    for (size_t i = 0; i < len; ++i) {
        int64_t elapsed_time_us = ((int64_t)timeout * 1000) - (int64_t)(time_us_64() - start_time_us);
        if (elapsed_time_us < 0) {
            if (errcode != NULL) {
                *errcode = 1;
            }
            return i;
        }

        int character = getchar_timeout_us((uint32_t)elapsed_time_us);
        if (character == PICO_ERROR_TIMEOUT) {
            if (errcode != NULL) {
                *errcode = 1;
            }
            return i;
        }

        buf[i] = (uint8_t)character;
    }

    if (errcode != NULL) {
        *errcode = 0;
    }
    return len;
}
