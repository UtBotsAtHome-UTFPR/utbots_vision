#include <Arduino.h>
#include <ESP32Servo.h>

#if __has_include(<micro_ros_platformio.h>)
  #include <micro_ros_platformio.h>
#else
  #include <micro_ros_arduino.h>   // Arduino IDE + micro_ros_arduino library
  #define MICRO_ROS_ARDUINO_LIB
#endif

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32_multi_array.h>

// ============================== CONFIGURATION ==============================

// --- Topic / units ---------------------------------------------------------
#define TOPIC_NAME        "/servo_angles"
#define INPUT_IN_DEGREES  false      // true if the server runs with publish_degrees:=true

// --- Pins ------------------------------------------------------------------
#define PAN_SERVO_PIN     13
#define TILT_SERVO_PIN    12
#define STATUS_LED_PIN    2          // on-board LED on most ESP32 dev boards

// --- Servo pulse range (check your servo's datasheet) ----------------------
// Pulse that corresponds to 0 deg and 180 deg of servo travel.
#define SERVO_MIN_US      500
#define SERVO_MAX_US      2500

// --- Mechanical calibration ------------------------------------------------
// Flip direction if the servo turns the wrong way relative to the URDF
// (pan +: counter-clockwise about Z / to the left, tilt +: about +Y).
#define PAN_INVERT        false
#define TILT_INVERT       false
// Trim (degrees) added after conversion, to fix a horn that isn't centred.
#define PAN_TRIM_DEG      0.0f
#define TILT_TRIM_DEG     0.0f

// --- Safety limits in the ROS frame (radians, from pan_tilt.urdf) ----------
#define PAN_MIN_RAD      -1.5708f
#define PAN_MAX_RAD       1.5708f
#define TILT_MIN_RAD     -1.2217f
#define TILT_MAX_RAD      1.2217f

// ===========================================================================

Servo panServo;
Servo tiltServo;

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Static buffers for the incoming message (micro-ROS does not allocate
// dynamically). The server sends 2 floats and an empty layout, but the
// layout buffers are sized so a publisher that fills them doesn't break
// deserialization.
static float data_buffer[8];
static std_msgs__msg__MultiArrayDimension dim_buffer[2];
static char dim_label_buffer[2][32];

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = (fn); (void)rc; }

// Run X at most every MS milliseconds
#define EXECUTE_EVERY_N_MS(MS, X) do {                       \
    static int64_t _t = -1;                                   \
    if (_t == -1) { _t = uxr_millis(); }                      \
    if (uxr_millis() - _t > (MS)) { X; _t = uxr_millis(); }   \
  } while (0)

// ------------------------------ Servo helpers ------------------------------

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Converts a ROS-frame angle (rad, 0 = centre) into a servo pulse width.
static int radToPulse(float rad, float minRad, float maxRad, bool invert, float trimDeg) {
  rad = clampf(rad, minRad, maxRad);
  if (invert) rad = -rad;
  float deg = 90.0f + rad * (180.0f / PI) + trimDeg;   // 90 deg = centre
  deg = clampf(deg, 0.0f, 180.0f);
  return SERVO_MIN_US + (int)((deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US) + 0.5f);
}

static void moveTo(float panRad, float tiltRad) {
  panServo.writeMicroseconds(radToPulse(panRad, PAN_MIN_RAD, PAN_MAX_RAD, PAN_INVERT, PAN_TRIM_DEG));
  tiltServo.writeMicroseconds(radToPulse(tiltRad, TILT_MIN_RAD, TILT_MAX_RAD, TILT_INVERT, TILT_TRIM_DEG));
}

// ------------------------------ Subscription -------------------------------

static void servo_angles_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *m =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  if (m->data.size < 2) return;   // need [pan, tilt]

  float pan  = m->data.data[0];
  float tilt = m->data.data[1];

  if (isnan(pan) || isnan(tilt)) return;

#if INPUT_IN_DEGREES
  // Server degree mode: 0..180 with 90 = centre -> back to radians around 0
  pan  = (pan  - 90.0f) * (PI / 180.0f);
  tilt = (tilt - 90.0f) * (PI / 180.0f);
#endif

  moveTo(pan, tilt);
  digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));   // blink on each command
}

// ---------------------------- micro-ROS entities ---------------------------

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "pan_tilt_esp32", "", &support));

  // Best-effort matches a 50 Hz command stream and copes with lossy links;
  // it still connects to the server's default (reliable) publisher.
  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      TOPIC_NAME));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &msg, &servo_angles_callback, ON_NEW_DATA));
  return true;
}

static void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

static void init_message_memory() {
  msg.data.data = data_buffer;
  msg.data.size = 0;
  msg.data.capacity = sizeof(data_buffer) / sizeof(data_buffer[0]);

  msg.layout.dim.data = dim_buffer;
  msg.layout.dim.size = 0;
  msg.layout.dim.capacity = 2;
  for (size_t i = 0; i < 2; i++) {
    dim_buffer[i].label.data = dim_label_buffer[i];
    dim_buffer[i].label.size = 0;
    dim_buffer[i].label.capacity = sizeof(dim_label_buffer[i]);
  }
  msg.layout.data_offset = 0;
}

// ---------------------------------- Setup ----------------------------------

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Servos: 50 Hz, then go to centre before ROS is up
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  tiltServo.attach(TILT_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  moveTo(0.0f, 0.0f);

  // Initialize Serial Transport for micro-ROS
  Serial.begin(115200);
  #ifdef MICRO_ROS_ARDUINO_LIB
    set_microros_transports();
  #else
    set_microros_serial_transports(Serial);
  #endif

  init_message_memory();
  delay(500);
}

// ----------------------------------- Loop ----------------------------------
// Reconnecting state machine: survives the agent being started late or
// restarted while the ESP32 keeps running.

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroy_entities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      digitalWrite(STATUS_LED_PIN, LOW);
      state = WAITING_AGENT;
      break;
  }
}
