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

#include "Coil.h"

// ——————————————————————————————
// 1) PWM setup & LED
// ——————————————————————————————
Coil coils(
  25, 26,  // GPIO pins for coil
  0, 1,    // LEDC channels
  1000,    // frequency
  8        // resolution
);

const int LED_PIN = 2;      // ESP32 built-in LED
volatile bool mode_enabled = false;

// ——————————————————————————————
// 2) ROS2 objects & type support
// ——————————————————————————————
rcl_subscription_t      pwm_subscriber;
std_msgs__msg__UInt8    pwm_msg;
rcl_subscription_t      mode_subscriber;
std_msgs__msg__Bool     mode_msg;

rclc_executor_t         executor;
rclc_support_t          support;
rcl_allocator_t         allocator;
rcl_node_t              node;
rcl_init_options_t      init_options;

const rosidl_message_type_support_t * ts_uint8 =
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
const rosidl_message_type_support_t * ts_bool =
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);

void error_loop() { while (1) delay(100); }
#define RCCHECK(fn)     { if ((fn) != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { (void)(fn); }

// ——————————————————————————————
// 3) Callbacks
// ——————————————————————————————
void mode_callback(const void * msgin)
{
  auto in = (const std_msgs__msg__Bool *)msgin;
  mode_enabled = in->data;
  // update coil & LED immediately
  if (mode_enabled) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    coils.off();
    digitalWrite(LED_PIN, LOW);
  }
  printf("Mode: %s\n", mode_enabled ? "ENABLED" : "DISABLED");
}

void pwm_callback(const void * msgin)
{
  auto in = (const std_msgs__msg__UInt8 *)msgin;
  if (mode_enabled) {
    coils.on(in->data);
    printf("PWM duty: %d\n", in->data);
  } else {
    coils.off();
    printf("PWM ignored (mode OFF)\n");
  }
}

// ——————————————————————————————
// 4) setup()
// ——————————————————————————————
void setup() {
  // 4.1) PWM init
  coils.begin();

  // 4.2) LED init
  pinMode(LED_PIN, OUTPUT);
  // quick blink test
  for(int i=0; i<3; i++){
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  digitalWrite(LED_PIN, LOW);

  // 4.3) Serial for micro-ROS
  _SERIAL_PORT.begin(_SERIAL_BAUDRATE);
  set_microros_serial_transports(_SERIAL_PORT);
  delay(2000);

  // 4.4) rclc init
  allocator    = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "break_node", "", &support));

  // 4.5) Subscribers
  RCCHECK(rclc_subscription_init_default(
    &mode_subscriber,
    &node,
    ts_bool,
    "mode"
  ));
  RCCHECK(rclc_subscription_init_default(
    &pwm_subscriber,
    &node,
    ts_uint8,
    "pwm_duty"
  ));

  // 4.6) Executor (2 subs)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &mode_subscriber, &mode_msg, &mode_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pwm_subscriber,  &pwm_msg,  &pwm_callback,  ON_NEW_DATA));
}

// ——————————————————————————————
// 5) loop()
// ——————————————————————————————
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
