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
const int COIL1_PIN = 25, COIL2_PIN = 26;
const int PWM_CH1   = 0,  PWM_CH2   = 1;
const int PWM_FREQ  = 1000;  // Hz
const int PWM_RES   = 8;     // bits

void coil_init() {
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcAttachPin(COIL1_PIN, PWM_CH1);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(COIL2_PIN, PWM_CH2);
}
void coil_on(int duty)  { ledcWrite(PWM_CH1, duty);  ledcWrite(PWM_CH2, duty); }
void coil_off()         { ledcWrite(PWM_CH1, 0);     ledcWrite(PWM_CH2, 0);   }

void subscription_callback(const void * msgin) {
  auto in = (const std_msgs__msg__UInt8 *)msgin;
  coil_on(in->data);               // 0 will turn it off
  printf("PWM duty set to: %d\n", in->data);
}

void setup() {
  // Step 1: Initialize PWM channels for both coils
  coil_init();
  coil_off();  // ensure coils start in the OFF state

  // Step 2: Start Serial port for micro-ROS transport
  _SERIAL_PORT.begin(_SERIAL_BAUDRATE);

  // Step 3: Tell micro-ROS to use this Serial port
  set_microros_serial_transports(_SERIAL_PORT);

  // Step 4: Wait for the Serial link and agent handshake
  delay(2000);

  // Step 5: Get the default allocator for ROS 2 objects
  allocator = rcl_get_default_allocator();

  // Step 6: Zero-initialize the ROS 2 init options
  init_options = rcl_get_zero_initialized_init_options();

  // Step 7: Initialize init options with the allocator
  RCCHECK(rcl_init_options_init(&init_options, allocator));

  // Step 8: Set the ROS 2 domain ID (must match agent/PC)
  RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));

  // Step 9: Initialize the rclc support structure with these options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // Step 10: Create the ROS 2 node
  RCCHECK(rclc_node_init_default(&node, node_name, name_space, &support));

  // Step 11: Create a subscription to the "pwm_duty" topic
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, type_support, topic_name));

  // Step 12: Initialize the executor with space for one handle
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // Step 13: Add the subscription callback to the executor
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &sub_msg,
    &subscription_callback,
    ON_NEW_DATA
  ));
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
