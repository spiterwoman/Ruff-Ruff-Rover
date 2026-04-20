#include <stdbool.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

static rcl_publisher_t g_heartbeat_pub;
static std_msgs__msg__Bool g_heartbeat_msg;

static void heartbeat_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }
    g_heartbeat_msg.data = !g_heartbeat_msg.data;
    (void)rcl_publish(&g_heartbeat_pub, &g_heartbeat_msg, NULL);
}

int main(void) {
#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    while (rmw_uros_ping_agent(1000, 1) != RCL_RET_OK) {
#ifdef PICO_DEFAULT_LED_PIN
        gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
#endif
        sleep_ms(500);
    }
#ifdef PICO_DEFAULT_LED_PIN
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t heartbeat_timer;
    rclc_executor_t executor;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_diag_node", "", &support);

    std_msgs__msg__Bool__init(&g_heartbeat_msg);
    g_heartbeat_msg.data = false;

    rclc_publisher_init_default(
        &g_heartbeat_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/pico/heartbeat"
    );

    rclc_timer_init_default(
        &heartbeat_timer,
        &support,
        RCL_MS_TO_NS(500),
        heartbeat_timer_cb
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &heartbeat_timer);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
    }

    return 0;
}
