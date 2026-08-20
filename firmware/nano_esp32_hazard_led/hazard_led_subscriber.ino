#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/u_int8.h>


rcl_subscription_t hazard_subscriber;
std_msgs__msg__UInt8 hazard_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


// If no hazard message arrives for this long,
// force the LED to CRITICAL red.
const unsigned long HAZARD_TIMEOUT_MS = 1000;

unsigned long last_hazard_message_ms = 0;
bool hazard_message_received = false;


#define RCCHECK(fn)                                \
  {                                                \
    rcl_ret_t temp_rc = fn;                        \
    if (temp_rc != RCL_RET_OK) {                   \
      error_loop();                                \
    }                                              \
  }

#define RCSOFTCHECK(fn)                            \
  {                                                \
    rcl_ret_t temp_rc = fn;                        \
    if (temp_rc != RCL_RET_OK) {                   \
    }                                              \
  }


// Nano ESP32 RGB LED is active-low:
// LOW  = channel ON
// HIGH = channel OFF
void set_rgb(bool red_on, bool green_on, bool blue_on)
{
  digitalWrite(LED_RED,   red_on   ? LOW : HIGH);
  digitalWrite(LED_GREEN, green_on ? LOW : HIGH);
  digitalWrite(LED_BLUE,  blue_on  ? LOW : HIGH);
}


void set_hazard_led(uint8_t level)
{
  switch (level)
  {
    case 0:
      // CLEAR -> Green
      set_rgb(false, true, false);
      break;

    case 1:
      // CAUTION -> Blue
      set_rgb(false, false, true);
      break;

    case 2:
      // WARNING -> Yellow = Red + Green
      set_rgb(true, true, false);
      break;

    case 3:
      // CRITICAL -> Red
      set_rgb(true, false, false);
      break;

    default:
      // Unknown value -> fail safe
      set_rgb(true, false, false);
      break;
  }
}


void error_loop()
{
  // Any micro-ROS initialization failure is fail-safe red.
  set_hazard_led(3);

  while (1)
  {
    delay(100);
  }
}


void hazard_callback(const void *msgin)
{
  const std_msgs__msg__UInt8 *msg =
    (const std_msgs__msg__UInt8 *)msgin;

  set_hazard_led(msg->data);

  last_hazard_message_ms = millis();
  hazard_message_received = true;
}


void setup()
{
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // Start in fail-safe state until ROS tells us otherwise.
  set_hazard_led(3);

  set_microros_transports();

  // Allow USB serial transport time to initialize.
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(
    rclc_support_init(
      &support,
      0,
      NULL,
      &allocator
    )
  );

  RCCHECK(
    rclc_node_init_default(
      &node,
      "hazard_led_node",
      "",
      &support
    )
  );

  RCCHECK(
    rclc_subscription_init_default(
      &hazard_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/hazard_level"
    )
  );

  RCCHECK(
    rclc_executor_init(
      &executor,
      &support.context,
      1,
      &allocator
    )
  );

  RCCHECK(
    rclc_executor_add_subscription(
      &executor,
      &hazard_subscriber,
      &hazard_msg,
      &hazard_callback,
      ON_NEW_DATA
    )
  );
}


void loop()
{
  RCSOFTCHECK(
    rclc_executor_spin_some(
      &executor,
      RCL_MS_TO_NS(50)
    )
  );

  // Local communications fail-safe:
  // if ROS hazard updates disappear, go red.
  if (
    !hazard_message_received ||
    (millis() - last_hazard_message_ms > HAZARD_TIMEOUT_MS)
  )
  {
    set_hazard_led(3);
  }

  delay(10);
}
