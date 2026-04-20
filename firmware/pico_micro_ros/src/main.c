#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#define PWM_LEFT_PIN 15
#define DIR_LEFT_PIN 14
#define PWM_RIGHT_PIN 16
#define DIR_RIGHT_PIN 17

#define ENC_LEFT_A_PIN 4
#define ENC_LEFT_B_PIN 13
#define ENC_RIGHT_A_PIN 2
#define ENC_RIGHT_B_PIN 3

#define I2C_PORT i2c0
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7
#define XSHUT_LEFT_PIN 8
#define XSHUT_RIGHT_PIN 9
#define TOF_DEFAULT_ADDR 0x29
#define TOF_LEFT_ADDR 0x30
#define TOF_RIGHT_ADDR 0x31

#define PWM_WRAP 999
#define PWM_CLKDIV 125.0f
#define MAX_WHEEL_SPEED_MPS 0.45f
#define WHEEL_BASE_M 0.23f
#define TICKS_PER_METER 820.0f
#define LEFT_MOTOR_SIGN 1.0f
#define RIGHT_MOTOR_SIGN -1.0f
#define DEADMAN_TIMEOUT_US 250000ULL
#define TOF_MIN_RANGE_M 0.02f
#define TOF_MAX_RANGE_M 2.00f
#define TOF_FOV_RAD 0.436332f
#define INFRARED_RADIATION_TYPE 1
#define I2C_TIMEOUT_US 5000

static volatile int32_t g_left_ticks = 0;
static volatile int32_t g_right_ticks = 0;
static bool g_prev_left_a = false;
static bool g_prev_right_a = false;

static float g_cmd_linear = 0.0f;
static float g_cmd_angular = 0.0f;
static uint64_t g_last_cmd_us = 0;

static float g_odom_x = 0.0f;
static float g_odom_y = 0.0f;
static float g_odom_yaw = 0.0f;
static int32_t g_prev_left_ticks = 0;
static int32_t g_prev_right_ticks = 0;
static uint64_t g_prev_odom_us = 0;

static bool g_left_sensor_ready = false;
static bool g_right_sensor_ready = false;

static rcl_subscription_t g_cmd_sub;
static rcl_publisher_t g_odom_pub;
static rcl_publisher_t g_left_range_pub;
static rcl_publisher_t g_right_range_pub;
static rcl_publisher_t g_heartbeat_pub;
static rcl_publisher_t g_wheel_state_pub;
static rcl_timer_t g_publish_timer;

static geometry_msgs__msg__Twist g_cmd_msg;
static nav_msgs__msg__Odometry g_odom_msg;
static sensor_msgs__msg__Range g_left_range_msg;
static sensor_msgs__msg__Range g_right_range_msg;
static std_msgs__msg__Bool g_heartbeat_msg;
static geometry_msgs__msg__Vector3 g_wheel_state_msg;

static void set_motor_output(float left, float right) {
    left *= LEFT_MOTOR_SIGN;
    right *= RIGHT_MOTOR_SIGN;

    if (left > 1.0f) {
        left = 1.0f;
    }
    if (left < -1.0f) {
        left = -1.0f;
    }
    if (right > 1.0f) {
        right = 1.0f;
    }
    if (right < -1.0f) {
        right = -1.0f;
    }

    gpio_put(DIR_LEFT_PIN, left >= 0.0f ? 1 : 0);
    gpio_put(DIR_RIGHT_PIN, right >= 0.0f ? 1 : 0);

    pwm_set_gpio_level(PWM_LEFT_PIN, (uint16_t)(fabsf(left) * PWM_WRAP));
    pwm_set_gpio_level(PWM_RIGHT_PIN, (uint16_t)(fabsf(right) * PWM_WRAP));
}

static void stop_motors(void) {
    set_motor_output(0.0f, 0.0f);
}

static void drive_from_command(void) {
    uint64_t now_us = time_us_64();
    if (now_us - g_last_cmd_us > DEADMAN_TIMEOUT_US) {
        stop_motors();
        return;
    }

    float left_velocity = g_cmd_linear - (g_cmd_angular * WHEEL_BASE_M * 0.5f);
    float right_velocity = g_cmd_linear + (g_cmd_angular * WHEEL_BASE_M * 0.5f);

    set_motor_output(left_velocity / MAX_WHEEL_SPEED_MPS, right_velocity / MAX_WHEEL_SPEED_MPS);
}

static void setup_motors(void) {
    gpio_init(DIR_LEFT_PIN);
    gpio_set_dir(DIR_LEFT_PIN, GPIO_OUT);
    gpio_init(DIR_RIGHT_PIN);
    gpio_set_dir(DIR_RIGHT_PIN, GPIO_OUT);

    gpio_set_function(PWM_LEFT_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM_RIGHT_PIN, GPIO_FUNC_PWM);

    uint left_slice = pwm_gpio_to_slice_num(PWM_LEFT_PIN);
    pwm_set_clkdiv(left_slice, PWM_CLKDIV);
    pwm_set_wrap(left_slice, PWM_WRAP);
    pwm_set_enabled(left_slice, true);

    uint right_slice = pwm_gpio_to_slice_num(PWM_RIGHT_PIN);
    pwm_set_clkdiv(right_slice, PWM_CLKDIV);
    pwm_set_wrap(right_slice, PWM_WRAP);
    pwm_set_enabled(right_slice, true);

    stop_motors();
}

static void setup_encoders(void) {
    gpio_init(ENC_LEFT_A_PIN);
    gpio_set_dir(ENC_LEFT_A_PIN, GPIO_IN);
    gpio_pull_up(ENC_LEFT_A_PIN);
    gpio_init(ENC_LEFT_B_PIN);
    gpio_set_dir(ENC_LEFT_B_PIN, GPIO_IN);
    gpio_pull_up(ENC_LEFT_B_PIN);
    gpio_init(ENC_RIGHT_A_PIN);
    gpio_set_dir(ENC_RIGHT_A_PIN, GPIO_IN);
    gpio_pull_up(ENC_RIGHT_A_PIN);
    gpio_init(ENC_RIGHT_B_PIN);
    gpio_set_dir(ENC_RIGHT_B_PIN, GPIO_IN);
    gpio_pull_up(ENC_RIGHT_B_PIN);

    // Seed initial A-channel state so first poll does not create a phantom tick.
    g_prev_left_a = gpio_get(ENC_LEFT_A_PIN);
    g_prev_right_a = gpio_get(ENC_RIGHT_A_PIN);
}

static void poll_encoders(void) {
    bool current_left_a = gpio_get(ENC_LEFT_A_PIN);
    bool current_right_a = gpio_get(ENC_RIGHT_A_PIN);

    if (current_left_a && !g_prev_left_a) {
        if (gpio_get(ENC_LEFT_B_PIN)) {
            g_left_ticks++;
        } else {
            g_left_ticks--;
        }
    }

    if (current_right_a && !g_prev_right_a) {
        if (gpio_get(ENC_RIGHT_B_PIN)) {
            g_right_ticks++;
        } else {
            g_right_ticks--;
        }
    }

    g_prev_left_a = current_left_a;
    g_prev_right_a = current_right_a;
}

static bool i2c_write_bytes(uint8_t addr, const uint8_t * data, size_t len) {
    int written = i2c_write_timeout_us(I2C_PORT, addr, data, len, false, I2C_TIMEOUT_US);
    return written == (int)len;
}

static bool i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t * data, size_t len) {
    int wrote_reg = i2c_write_timeout_us(I2C_PORT, addr, &reg, 1, true, I2C_TIMEOUT_US);
    if (wrote_reg != 1) {
        return false;
    }
    int read_len = i2c_read_timeout_us(I2C_PORT, addr, data, len, false, I2C_TIMEOUT_US);
    return read_len == (int)len;
}

static bool vl53_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t payload[2] = {reg, value};
    return i2c_write_bytes(addr, payload, 2);
}

static bool vl53_set_address(uint8_t current_addr, uint8_t new_addr) {
    return vl53_write_reg(current_addr, 0x8A, new_addr & 0x7F);
}

static bool vl53_read_distance(uint8_t addr, float * range_m) {
    uint8_t data[2] = {0};
    if (!vl53_write_reg(addr, 0x00, 0x01)) {
        return false;
    }
    sleep_ms(50);
    if (!i2c_read_bytes(addr, 0x1E, data, 2)) {
        return false;
    }
    uint16_t distance_mm = ((uint16_t)data[0] << 8) | data[1];
    *range_m = (float)distance_mm / 1000.0f;
    return true;
}

static bool setup_tof_sensors(void) {
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    gpio_init(XSHUT_LEFT_PIN);
    gpio_set_dir(XSHUT_LEFT_PIN, GPIO_OUT);
    gpio_init(XSHUT_RIGHT_PIN);
    gpio_set_dir(XSHUT_RIGHT_PIN, GPIO_OUT);

    gpio_put(XSHUT_LEFT_PIN, 0);
    gpio_put(XSHUT_RIGHT_PIN, 0);
    sleep_ms(10);

    gpio_put(XSHUT_LEFT_PIN, 1);
    sleep_ms(10);
    g_left_sensor_ready = vl53_set_address(TOF_DEFAULT_ADDR, TOF_LEFT_ADDR);

    gpio_put(XSHUT_RIGHT_PIN, 1);
    sleep_ms(10);
    g_right_sensor_ready = vl53_set_address(TOF_DEFAULT_ADDR, TOF_RIGHT_ADDR);

    return g_left_sensor_ready && g_right_sensor_ready;
}

static void fill_stamp(builtin_interfaces__msg__Time * stamp) {
    uint64_t now_us = time_us_64();
    stamp->sec = (int32_t)(now_us / 1000000ULL);
    stamp->nanosec = (uint32_t)((now_us % 1000000ULL) * 1000ULL);
}

static void update_odometry(void) {
    uint64_t now_us = time_us_64();
    if (g_prev_odom_us == 0) {
        g_prev_odom_us = now_us;
        g_prev_left_ticks = g_left_ticks;
        g_prev_right_ticks = g_right_ticks;
        return;
    }

    float dt = (float)(now_us - g_prev_odom_us) / 1000000.0f;
    if (dt <= 0.0f) {
        return;
    }

    int32_t current_left = g_left_ticks;
    int32_t current_right = g_right_ticks;

    float left_delta = (float)(current_left - g_prev_left_ticks) / TICKS_PER_METER;
    float right_delta = (float)(current_right - g_prev_right_ticks) / TICKS_PER_METER;
    float distance = 0.5f * (left_delta + right_delta);
    float yaw_delta = (right_delta - left_delta) / WHEEL_BASE_M;

    float heading_mid = g_odom_yaw + (yaw_delta * 0.5f);
    g_odom_x += distance * cosf(heading_mid);
    g_odom_y += distance * sinf(heading_mid);
    g_odom_yaw += yaw_delta;

    fill_stamp(&g_odom_msg.header.stamp);
    g_odom_msg.pose.pose.position.x = g_odom_x;
    g_odom_msg.pose.pose.position.y = g_odom_y;
    g_odom_msg.pose.pose.position.z = 0.0;
    g_odom_msg.pose.pose.orientation.x = 0.0;
    g_odom_msg.pose.pose.orientation.y = 0.0;
    g_odom_msg.pose.pose.orientation.z = sinf(g_odom_yaw * 0.5f);
    g_odom_msg.pose.pose.orientation.w = cosf(g_odom_yaw * 0.5f);
    g_odom_msg.twist.twist.linear.x = distance / dt;
    g_odom_msg.twist.twist.angular.z = yaw_delta / dt;

    g_wheel_state_msg.x = (float)current_left / TICKS_PER_METER;
    g_wheel_state_msg.y = (float)current_right / TICKS_PER_METER;
    g_wheel_state_msg.z = 0.0f;

    g_prev_odom_us = now_us;
    g_prev_left_ticks = current_left;
    g_prev_right_ticks = current_right;
}

static void update_ranges(void) {
    float left_range = TOF_MAX_RANGE_M;
    float right_range = TOF_MAX_RANGE_M;

    if (g_left_sensor_ready) {
        float measured = TOF_MAX_RANGE_M;
        if (vl53_read_distance(TOF_LEFT_ADDR, &measured)) {
            left_range = measured;
        }
    }

    if (g_right_sensor_ready) {
        float measured = TOF_MAX_RANGE_M;
        if (vl53_read_distance(TOF_RIGHT_ADDR, &measured)) {
            right_range = measured;
        }
    }

    fill_stamp(&g_left_range_msg.header.stamp);
    fill_stamp(&g_right_range_msg.header.stamp);
    g_left_range_msg.range = left_range;
    g_right_range_msg.range = right_range;
}

static void cmd_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    g_cmd_linear = msg->linear.x;
    g_cmd_angular = msg->angular.z;
    g_last_cmd_us = time_us_64();
}

static void publish_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }

    drive_from_command();
    update_odometry();
    update_ranges();

    g_heartbeat_msg.data = true;

    rcl_publish(&g_odom_pub, &g_odom_msg, NULL);
    rcl_publish(&g_left_range_pub, &g_left_range_msg, NULL);
    rcl_publish(&g_right_range_pub, &g_right_range_msg, NULL);
    rcl_publish(&g_heartbeat_pub, &g_heartbeat_msg, NULL);
    rcl_publish(&g_wheel_state_pub, &g_wheel_state_msg, NULL);
}

static void init_messages(void) {
    nav_msgs__msg__Odometry__init(&g_odom_msg);
    sensor_msgs__msg__Range__init(&g_left_range_msg);
    sensor_msgs__msg__Range__init(&g_right_range_msg);
    std_msgs__msg__Bool__init(&g_heartbeat_msg);
    geometry_msgs__msg__Vector3__init(&g_wheel_state_msg);
    geometry_msgs__msg__Twist__init(&g_cmd_msg);

    rosidl_runtime_c__String__assign(&g_odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&g_odom_msg.child_frame_id, "base_link");
    rosidl_runtime_c__String__assign(&g_left_range_msg.header.frame_id, "front_left_tof");
    rosidl_runtime_c__String__assign(&g_right_range_msg.header.frame_id, "front_right_tof");

    g_left_range_msg.radiation_type = INFRARED_RADIATION_TYPE;
    g_left_range_msg.field_of_view = TOF_FOV_RAD;
    g_left_range_msg.min_range = TOF_MIN_RANGE_M;
    g_left_range_msg.max_range = TOF_MAX_RANGE_M;

    g_right_range_msg.radiation_type = INFRARED_RADIATION_TYPE;
    g_right_range_msg.field_of_view = TOF_FOV_RAD;
    g_right_range_msg.min_range = TOF_MIN_RANGE_M;
    g_right_range_msg.max_range = TOF_MAX_RANGE_M;
}

int main(void) {
    g_last_cmd_us = time_us_64();

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    while (rmw_uros_ping_agent(1000, 1) != RCL_RET_OK) {
        sleep_ms(500);
    }

    // Defer hardware initialization until the transport handshake succeeds.
    // This avoids startup failures from motor/encoder wiring side effects.
    setup_motors();
    stop_motors();
    setup_encoders();

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    init_messages();

    rclc_publisher_init_default(
        &g_odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"
    );

    rclc_publisher_init_default(
        &g_left_range_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/range/front_left"
    );

    rclc_publisher_init_default(
        &g_right_range_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/range/front_right"
    );

    rclc_publisher_init_default(
        &g_heartbeat_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/pico/heartbeat"
    );

    rclc_publisher_init_default(
        &g_wheel_state_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "/wheel_state"
    );

    rclc_subscription_init_default(
        &g_cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    );

    rclc_timer_init_default(
        &g_publish_timer,
        &support,
        RCL_MS_TO_NS(50),
        publish_callback
    );

    // Bring up ToF sensors after the ROS session is established so sensor
    // issues cannot prevent heartbeat/odom connectivity.
    setup_tof_sensors();

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &g_cmd_sub, &g_cmd_msg, &cmd_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &g_publish_timer);

    while (true) {
        poll_encoders();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
    }

    return 0;
}
