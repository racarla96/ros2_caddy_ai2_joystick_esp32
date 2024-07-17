

/**
 * @file main.cpp
 * @brief This file contains the main code for the ROS2 Caddy AI2 Joystick ESP32 Radio Bridge.
 * 
 * The code initializes the necessary entities for communication with a micro-ROS agent and a joystick.
 * It reads the PWM values from the joystick channels, saturates and normalizes them, and publishes the values as a sensor_msgs/Joy message.
 * The code also handles the connection and disconnection with the micro-ROS agent.
 * 
 * The main loop of the code checks the state of the connection with the agent and performs the necessary actions accordingly.
 * 
 * This code is specific to the Arduino framework with serial transport.
 */
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

// Definir las constantes para los canales
const int C_CH1[] = {1990, 1205, 1590, 1605};
const int C_CH2[] = {1765, 1070, 1410, 1425};
const int C_CH3[] = {1760, 1100};
const int C_CH4[] = {1840, 1062, 1440, 1462};
const int C_CH5[] = {1900, 1100};
const int C_CH6[] = {1920, 1120};

// Variables para los valores de los canales
int raw_CH1, raw_CH2, raw_CH3, raw_CH4, raw_CH5, raw_CH6;

float m1, m2, m3, m4, m5, m6;
int sat1_CH1, sat1_CH2, sat1_CH4;
int sat2_CH1, sat2_CH2, sat2_CH4;
int sat_CH3, sat_CH5, sat_CH6;
float u1_CH1, u2_CH1, u1_CH2, u2_CH2, u1_CH4, u2_CH4;
float u_CH1, u_CH2, u_CH3, u_CH4, u_CH5, u_CH6;

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

int period_ms = 20;

struct timespec ts;
char frame_id[] = "joy_link";
rcl_publisher_t publisher;
sensor_msgs__msg__Joy msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Error handle loop
void error_loop() {
  bool led_state = true;
  while(1){
    if(led_state) {digitalWrite(LED, HIGH); led_state = false;}
    else {digitalWrite(LED, LOW); led_state = true;}
    delay(100);
  }
}

float calcularPendiente(int maxVal, int minVal) {
    return abs(maxVal - minVal);
}

int saturar(int valor, int maxVal, int minVal) {
    valor = min(valor, maxVal);
    valor = max(valor, minVal);
    return valor;
}

float normalizar(int valor, int offset, float pendiente) {
    return (1.0 / pendiente) * (valor - offset);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {

    clock_gettime(CLOCK_REALTIME, &ts);
    msg.header.stamp.sec = ts.tv_sec;
    msg.header.stamp.nanosec = ts.tv_nsec;

    raw_CH1 = ch1.getDutyCicle_us();
    raw_CH2 = ch2.getDutyCicle_us();
    raw_CH3 = ch3.getDutyCicle_us();
    raw_CH4 = ch4.getDutyCicle_us();
    raw_CH5 = ch5.getDutyCicle_us();
    raw_CH6 = ch6.getDutyCicle_us();

    // Saturar y normalizar los valores para CH1
    sat1_CH1 = saturar(raw_CH1, C_CH1[0], C_CH1[3]);
    sat2_CH1 = saturar(raw_CH1, C_CH1[2], C_CH1[1]);
    u1_CH1 = normalizar(sat1_CH1, C_CH1[3], m1);
    u2_CH1 = normalizar(sat2_CH1, C_CH1[2], m1);
    u_CH1 = -1 * (u1_CH1 + u2_CH1);

    // Saturar y normalizar los valores para CH2
    sat1_CH2 = saturar(raw_CH2, C_CH2[0], C_CH2[3]);
    sat2_CH2 = saturar(raw_CH2, C_CH2[2], C_CH2[1]);
    u1_CH2 = normalizar(sat1_CH2, C_CH2[3], m2);
    u2_CH2 = normalizar(sat2_CH2, C_CH2[2], m2);
    u_CH2 = u1_CH2 + u2_CH2;

    // Saturar y normalizar los valores para CH3
    sat_CH3 = saturar(raw_CH3, C_CH3[0], C_CH3[1]);
    u_CH3 = normalizar(sat_CH3, C_CH3[1], m3);

    // Saturar y normalizar los valores para CH4
    sat1_CH4 = saturar(raw_CH4, C_CH4[0], C_CH4[3]);
    sat2_CH4 = saturar(raw_CH4, C_CH4[2], C_CH4[1]);
    u1_CH4 = normalizar(sat1_CH4, C_CH4[3], m4);
    u2_CH4 = normalizar(sat2_CH4, C_CH4[2], m4);
    u_CH4 = u1_CH4 + u2_CH4;

    // Saturar y normalizar los valores para CH5
    sat_CH5 = saturar(raw_CH5, C_CH5[0], C_CH5[1]);
    u_CH5 = normalizar(sat_CH5, C_CH5[1], m5);

    // Saturar y normalizar los valores para CH6
    sat_CH6 = saturar(raw_CH6, C_CH6[0], C_CH6[1]);
    u_CH6 = normalizar(sat_CH6, C_CH6[1], m6);

    msg.axes.data[0] = u_CH1;
    msg.axes.data[1] = u_CH2;
    msg.axes.data[2] = u_CH3;
    msg.axes.data[3] = u_CH4;
    msg.axes.data[4] = u_CH5;
    msg.axes.data[5] = u_CH6;

    msg.buttons.data[0] = ch1.isEnable();

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

bool create_entities() {
  if (!ch1.begin(CH1)) return false;
  if (!ch2.begin(CH2)) return false;
  if (!ch3.begin(CH3)) return false;
  if (!ch4.begin(CH4)) return false;
  if (!ch5.begin(CH5)) return false;
  if (!ch6.begin(CH6)) return false;

  m1 = calcularPendiente(C_CH1[0], C_CH1[3]);
  m2 = calcularPendiente(C_CH2[0], C_CH2[3]);
  m3 = calcularPendiente(C_CH3[0], C_CH3[1]);
  m4 = calcularPendiente(C_CH4[0], C_CH4[3]);
  m5 = calcularPendiente(C_CH5[0], C_CH5[1]);
  m6 = calcularPendiente(C_CH6[0], C_CH6[1]);

  msg.header.frame_id.data = frame_id;
  msg.header.frame_id.size = strlen(msg.header.frame_id.data);

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
    "joy")); // micro_ros_joystick_esp32_radio_bridge_node_publisher

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
  static int32_t buttons[1];
  msg.buttons.capacity = 1;
  msg.buttons.data = buttons;
  msg.buttons.size = 1;

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  set_microros_serial_transports(Serial);
  delay(2000);

  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(4000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(period_ms, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(period_ms));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED, 1);
  } else {
    digitalWrite(LED, 0);
  }
}