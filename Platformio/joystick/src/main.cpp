#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <string.h>

#define LED 2

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/joy.h>

#include "PWMReader.hpp"

#define CH1 27 
#define CH2 26
#define CH3 25
#define CH4 33
#define CH5 32
#define CH6 35

PWMReader ch1;
PWMReader ch2;
PWMReader ch3;
PWMReader ch4;
PWMReader ch5;
PWMReader ch6;

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

int period_ms = 20;

struct timespec tv = {0};
String frame_id = "joy_link";
rcl_publisher_t publisher;
sensor_msgs__msg__Joy msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  bool state = true;
  while(1){
    if(state) {digitalWrite(LED, HIGH); state = false;}
    else {digitalWrite(LED, LOW); state = true;}
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    clock_gettime(0, &tv);
    msg.header.stamp.sec = tv.tv_sec;
    msg.header.stamp.nanosec = tv.tv_nsec;
    //msg.header.frame_id = frame_id;
    
    msg.axes.data[0] = ch1.getDutyCicle_us();
    msg.axes.data[1] = ch2.getDutyCicle_us();
    msg.axes.data[2] = ch3.getDutyCicle_us();
    msg.axes.data[3] = ch4.getDutyCicle_us();
    msg.axes.data[4] = ch5.getDutyCicle_us();
    msg.axes.data[5] = ch6.getDutyCicle_us();

    msg.buttons.data[0] = ch1.isEnable();
    msg.buttons.data[1] = ch2.isEnable();
    msg.buttons.data[2] = ch3.isEnable();
    msg.buttons.data[3] = ch4.isEnable();
    msg.buttons.data[4] = ch5.isEnable();
    msg.buttons.data[5] = ch6.isEnable();  

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  if (!ch1.begin(CH1)) error_loop();
  if (!ch2.begin(CH2)) error_loop();
  if (!ch3.begin(CH3)) error_loop();
  if (!ch4.begin(CH4)) error_loop();
  if (!ch5.begin(CH5)) error_loop();
  if (!ch6.begin(CH6)) error_loop();

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_joystick_esp32_radio_bridge_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "joystick")); // micro_ros_joystick_esp32_radio_bridge_node_publisher

  // create timer,
  const unsigned int timer_period = period_ms; // ms
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_period),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // initialize message
  static float axes[6];
  msg.axes.capacity = 6;
  msg.axes.data = axes;
  msg.axes.size = 6;
  static int32_t buttons[6];
  msg.buttons.capacity = 6;
  msg.buttons.data = buttons;
  msg.buttons.size = 6;
}

void loop() {
  delay(period_ms/2);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(period_ms)));
}