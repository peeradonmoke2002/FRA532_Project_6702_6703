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

#include "Coil.h"

// — ROS 2 objects
rcl_subscription_t      subscriber;
std_msgs__msg__UInt8    sub_msg;
rclc_executor_t         executor;
rclc_support_t          support;
rcl_allocator_t         allocator;
rcl_node_t              node;
rcl_init_options_t      init_options;

const char * node_name    = "break_node";
const char * topic_name   = "pwm_duty";
const char * name_space   = "";
size_t       domain_id    = 0;
const rosidl_message_type_support_t * type_support =
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);

void error_loop() { while (1) delay(100); }
#define RCCHECK(fn)     { if ((fn) != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { (void)(fn); }

// — PWM setup
// Coil(pin1, pin2, ch1, ch2,      freq, resolution)
Coil coils(
  25,   // pin1:    GPIO25 for coil #1 PWM output
  26,   // pin2:    GPIO26 for coil #2 PWM output
  0,    // ch1:     LEDC channel 0 (assigned to pin1)
  1,    // ch2:     LEDC channel 1 (assigned to pin2)
  1000, // freq:    PWM frequency in Hz
  8     // res:     PWM resolution in bits (2^8 → 0–255)
);



void subscription_callback(const void * msgin) {
  auto in = (const std_msgs__msg__UInt8 *)msgin;
  coils.on(in->data);               // 0 will turn it off
  printf("PWM duty set to: %d\n", in->data);
}

void setup() {
  // 1) Initialize PWM channels and ensure OFF
  coils.begin();

  // 2) Start serial for micro-ROS
  _SERIAL_PORT.begin(_SERIAL_BAUDRATE);
  set_microros_serial_transports(_SERIAL_PORT);
  delay(2000);

  // 3) ROS 2 init
  allocator    = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, node_name, name_space, &support));
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, type_support, topic_name));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
