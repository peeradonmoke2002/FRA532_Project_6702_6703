// src/amt103_ros_node.cpp
#include <Arduino.h>
#include <micro_ros_platformio.h>

#define RMW_UROS_TRANSPORT_SERIAL
#define _SERIAL_PORT       Serial
#define _SERIAL_BAUDRATE   115200

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int64.h>

#include "Encoder.h"
#include "Coil.h"

// ——————————————————————————————
// 1) PWM & Encoder Setup
// ——————————————————————————————
Coil coils(25, 26, 0, 1, 1000, 1000, 8);
Encoder encoder(32, 33);  // AMT103 on GPIO32/33
const int LED_PIN = 2;    // Built-in LED
volatile bool mode_enabled = false;

// ——————————————————————————————
// 2) ROS2 Objects & Type Support
// ——————————————————————————————
rcl_subscription_t      pwm_subscriber;
std_msgs__msg__UInt8    pwm_msg;
rcl_subscription_t      mode_subscriber;
std_msgs__msg__Bool     mode_msg;
rcl_publisher_t         encoder_publisher;
std_msgs__msg__Int64    encoder_msg;
rcl_timer_t             publish_timer;

rclc_executor_t         executor;
rclc_support_t          support;
rcl_allocator_t         allocator;
rcl_node_t              node;
rcl_init_options_t      init_options;

const rosidl_message_type_support_t * ts_uint8  = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
const rosidl_message_type_support_t * ts_bool   = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
const rosidl_message_type_support_t * ts_int64  = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64);


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}
// ——————————————————————————————
// 3) Callbacks
// ——————————————————————————————
void mode_callback(const void * msgin)
{
  auto in = (const std_msgs__msg__Bool *)msgin;
  mode_enabled = in->data;
  if (mode_enabled) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    coils.off();
    digitalWrite(LED_PIN, LOW);
  }
}

void pwm_callback(const void * msgin)
{
  auto in = (const std_msgs__msg__UInt8 *)msgin;
  if (mode_enabled) {
    coils.on(in->data);
  } else {
    coils.off();
  }
}

// Timer callback to publish encoder count at 100 Hz
void timer_callback(rcl_timer_t * timer, int64_t last_call)
{
  // RCLC_UNUSED(last_call);
  if (timer != NULL) {
    encoder_msg.data = encoder.getCount();
    RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));
  }
}

// ——————————————————————————————
// 4) Setup
// ——————————————————————————————
void setup() {
  // Initialize hardware
  coils.begin();
  encoder.begin();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Blink LED
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  // Initialize micro-ROS transport
  _SERIAL_PORT.begin(_SERIAL_BAUDRATE);
  set_microros_serial_transports(_SERIAL_PORT);
  delay(2000);

  // Initialize rclc
  allocator    = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "amt103_node", "", &support));

  // Create subscribers
  RCCHECK(rclc_subscription_init_default(&mode_subscriber, &node, ts_bool,  "mode"));
  RCCHECK(rclc_subscription_init_default(&pwm_subscriber,  &node, ts_uint8, "pwm_duty"));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(&encoder_publisher, &node, ts_int64, "encoder_count"));

  // Create timer for publishing at 100 Hz (every 10 ms)
  RCCHECK(rclc_timer_init_default(&publish_timer, &support, RCL_MS_TO_NS(10), timer_callback));

  // Initialize executor with 3 handles: mode, pwm, timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &mode_subscriber, &mode_msg, &mode_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pwm_subscriber,  &pwm_msg,  &pwm_callback,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));

  encoder_msg.data = 0;  // Initialize encoder message
}

// ——————————————————————————————
// 5) Loop
// ——————————————————————————————
void loop() {
  // Spin executor to process subscriptions and timer
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
