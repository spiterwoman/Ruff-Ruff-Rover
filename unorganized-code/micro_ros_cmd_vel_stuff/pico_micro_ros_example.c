#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>
#include <std_msgs/msg/string.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h" // Ensure this uses stdio for USB
#include "pico/cyw43_arch.h"      // Required for Pico W LED
#include "hardware/pwm.h"

#define PWM1_PIN 15
#define PWM2_PIN 16
#define DIR1_PIN 14
#define DIR2_PIN 17
#define MOTOR_SPEED 800 // Out of 1000 (80% speed)

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_publisher_t debug_publisher;
std_msgs__msg__String debug_msg;

void setup_motors()
{
    // 1. Initialize Direction Pins
    gpio_init(DIR1_PIN);
    gpio_set_dir(DIR1_PIN, GPIO_OUT);
    gpio_init(DIR2_PIN);
    gpio_set_dir(DIR2_PIN, GPIO_OUT);

    // 2. Initialize PWM Pins
    gpio_set_function(PWM1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM2_PIN, GPIO_FUNC_PWM);

    // 3. Configure Slice for PWM1 (Pin 15)
    uint slice1 = pwm_gpio_to_slice_num(PWM1_PIN);
    pwm_set_clkdiv(slice1, 125.0f); // 1MHz clock
    pwm_set_wrap(slice1, 999);      // 1kHz frequency
    pwm_set_enabled(slice1, true);

    // 4. Configure Slice for PWM2 (Pin 16)
    uint slice2 = pwm_gpio_to_slice_num(PWM2_PIN);
    pwm_set_clkdiv(slice2, 125.0f); // 1MHz clock
    pwm_set_wrap(slice2, 999);      // 1kHz frequency
    pwm_set_enabled(slice2, true);    
}

// Callback: Blinks the Pico W LED when a message arrives
void subscription_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;

    // 2. Handle Direction and Scaling
    // Assume max speed of 1.0 m/s maps to our wrap value of 999
    uint16_t pwm1_value = 0;
    uint16_t pwm2_value = 0;

    // Calculate target speeds for each side
    // Standard differential drive formula:
    float left_speed = linear_x - angular_z;
    float right_speed = linear_x + angular_z;

    // 2. Set Left Motor (PWM1/DIR1)
    if (left_speed >= 0) 
    {
        gpio_put(DIR1_PIN, 1);
        pwm1_value = (uint16_t)(left_speed * 999);
    } 
    else 
    {
        gpio_put(DIR1_PIN, 0);
        pwm1_value = (uint16_t)(left_speed * 999);
    }

    // 3. Set Right Motor (PWM2/DIR2)
    if (right_speed >= 0) 
    {
        gpio_put(DIR2_PIN, 1);
        pwm2_value = (uint16_t)(right_speed * 999);
    } 
    else 
    {
        gpio_put(DIR2_PIN, 0);
        pwm2_value = (uint16_t)(right_speed * 999);
    }

    // 3. Constrain PWM to your max wrap (999)
    if (pwm1_value > 999) 
    {
        pwm1_value = 999;
    }
    if (pwm2_value > 999)
    {
        pwm2_value = 999;
    }

    // 4. Apply to Motors
    pwm_set_gpio_level(PWM1_PIN, pwm1_value);
    pwm_set_gpio_level(PWM2_PIN, pwm2_value);

    // Blink LED to show activity
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    
    // Debug Publisher 
    const char * text = "Moving";
    strcpy(debug_msg.data.data, text);
    debug_msg.data.size = strlen(text);
    rcl_ret_t ret = rcl_publish(&debug_publisher, &debug_msg, NULL);
    
    if (ret != RCL_RET_OK) 
    { 
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); 
        pwm_set_gpio_level(PWM1_PIN, 0);
        pwm_set_gpio_level(PWM2_PIN, 0);
    } 
}

int main()
{
    // 1. Initialize all stdio (USB Serial)
    //stdio_init_all();

    // 2. Initialize Pico W WiFi chip (to access the LED)
    if (cyw43_arch_init()) {
        return -1;
    }

    setup_motors();

    // 3. Set up micro-ROS transport
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;
    

    // 4. Try to connect to agent, but DON'T exit if it fails
    // This keeps the USB port alive so the agent can find it later
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    
    // Blink LED to show it's searching for the agent
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    
    if (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK)
    {
        // If agent not found, we don't return. We stay in the loop.
        // This prevents the USB port from disappearing.
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    // 5. Standard micro-ROS Init
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    // Initialize message memory
    debug_msg.data.data = (char * ) malloc(100 * sizeof(char));
    debug_msg.data.size = 0;
    debug_msg.data.capacity = 100;

    // Initialize the publisher
    rclc_publisher_init_default(
        &debug_publisher, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), 
        "pico_debug"
    );

    
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
    
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    while (true)
    {
        // spin_some keeps the USB stack responsive
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}

