#include <Arduino.h>

//MICROROS
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcutils/logging.h>
#include <rclc/executor.h>
#include <rclc/subscription.h>
#include <rcl/time.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>
//INTERFACES
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <sb_msgs/msg/hivepoker_status.h>
#include "rosidl_runtime_c/primitives_sequence.h"
//I2C EXTENDER
#include <Wire.h>
#include <Adafruit_PCF8575.h>
//STEPPERS
#include <ESP_FlexyStepper.h>
//MISC
#include <pthread.h>
#include <logger.h>
//HARDWARE
#include <new_pin_definitions.h>
#include <parameters.h>
#include <ros_parameters.h>


#define STR_SIZE 500


//STEPPER DEFINITIONS ===============================================================================================================================================
ESP_FlexyStepper x1_driver;
ESP_FlexyStepper y1_driver;
bool x1_motor_asleep = true, y1_motor_asleep = true, x2_motor_asleep = true, y2_motor_asleep = true;
bool x1_driver_fault = false, y1_driver_fault = false, x2_driver_fault = false, y2_driver_fault = false;


// I2C DEFINITIONS ===============================================================================================================================================
Adafruit_PCF8575 pcf8575;

//MICROROS DEFINITIONS ===============================================================================================================================================
rcl_publisher_t state_publisher, log_pub, status_pub;
rclc_executor_t executor;
rcl_subscription_t target_position_sub, vicon_position_sub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t state_timer, health_timer;
rcl_clock_t sys_clock;
const unsigned int state_timer_timeout = 50; //ms
const unsigned int health_timer_timeout = 1000; //ms
std_msgs__msg__String log_msg;
sensor_msgs__msg__JointState state_msg, target_msg;
sb_msgs__msg__HivepokerStatus status_msg;
geometry_msgs__msg__PoseStamped vicon_msg;
bool ready = false;





//ENDSTOP DEFINITIONS ===============================================================================================================================================
volatile bool left_endstop_triggered = false;
TaskHandle_t xLeftEndstopTaskHandle = NULL;
volatile bool right_endstop_triggered = false;
TaskHandle_t xRightEndstopTaskHandle = NULL;
volatile bool y_endstop_triggered = false;
TaskHandle_t xYEndstopTaskHandle = NULL;
bool already_stopped_x, already_stopped_y = false;



//MOVING UTILS ===============================================================================================================================================
float start_x = 0.0;
float start_y = 0.0;
float whycode_offset_x = 0.0;
float whycode_offset_y = 0.0;
bool first_whycode_message = false;





//MOVING FUNCTIONS ===============================================================================================================================================
bool move_x_arm1(float x) {

  if (!pcf8575.digitalWrite(X1_SLEEP_PIN, HIGH)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
  x1_motor_asleep = false;

  if (x<=0 && left_endstop_triggered) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Left movement not allowed");
    if (!pcf8575.digitalWrite(X1_SLEEP_PIN, LOW)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
    x1_motor_asleep = true;
    return false;
  }

  if (x>=420.0 && right_endstop_triggered) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Right movement not allowed");
    if (!pcf8575.digitalWrite(X1_SLEEP_PIN, LOW)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
    x1_motor_asleep = true;
    return false;
  }

  status_msg.status = sb_msgs__msg__HivepokerStatus__MOVING;
  x1_driver.setTargetPositionInMillimeters(-x);

  do {

    if (left_endstop_triggered && !already_stopped_x) {
      hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING, "Touched Left Endstop, Stopping");
      x1_driver.emergencyStop();
      x1_driver.setCurrentPositionInMillimeters(0.0);
      x1_driver.setCurrentPositionInSteps(0);
      x1_driver.setCurrentPositionInRevolutions(0);
      already_stopped_x = true;
      break;
    }  
    
    
    if (right_endstop_triggered && !already_stopped_x) {
      hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING, "Touched Right Endstop, Stopping");
      x1_driver.emergencyStop();
      already_stopped_x = true;
      break;
    }

    if (state_msg.position.data[0] * 1000 > 3.0 && left_endstop_triggered) {left_endstop_triggered = false; already_stopped_x = false;}
    if (state_msg.position.data[0] * 1000 < 420.0 && right_endstop_triggered) {right_endstop_triggered = false; already_stopped_x = false;}

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  } while (
    abs(x1_driver.getDistanceToTargetSigned()) > 0.0
  );

  if (!pcf8575.digitalWrite(X1_SLEEP_PIN, LOW)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
  x1_motor_asleep = true;
  status_msg.status = sb_msgs__msg__HivepokerStatus__IDLE;
  return true;

}



bool move_y_arm1(float y) {

  pcf8575.digitalWrite(Y1_SLEEP_PIN, HIGH);
  y1_motor_asleep = false;


  if (y<Y_MIN_VALUE_MM) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Down movement not allowed");
    pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);
    y1_motor_asleep = true;
    return false;
  }

  if (y>=Y_MAX_VALUE_MM) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Up movement not allowed");
    pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);
    y1_motor_asleep = true;
    return false;
  }

  status_msg.status = sb_msgs__msg__HivepokerStatus__MOVING;
  y1_driver.setTargetPositionInMillimeters(-y);

  do {

    if (y_endstop_triggered && !already_stopped_y) {
      hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING, "Touched Y Endstop, Stopping");
      y1_driver.emergencyStop();
      y1_driver.setCurrentPositionInMillimeters(0.0);
      y1_driver.setCurrentPositionInSteps(0);
      y1_driver.setCurrentPositionInRevolutions(0);
      already_stopped_y = true;
      break;
    }      


    if (state_msg.position.data[1] * 1000 > 3.0 && y_endstop_triggered) {y_endstop_triggered = false; already_stopped_y = false;}
    

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  } while (
    abs(y1_driver.getDistanceToTargetSigned()) > 0.0
  );

  pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);
  y1_motor_asleep = true;
  status_msg.status = sb_msgs__msg__HivepokerStatus__IDLE;
  return true;

}



void homing_y() {

    if (!pcf8575.digitalWrite(Y1_SLEEP_PIN, HIGH)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
    y1_motor_asleep = false;
    status_msg.status = sb_msgs__msg__HivepokerStatus__MOVING;
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Homing Y1 axis");
    y1_driver.setTargetPositionInMillimeters(y1_driver.getCurrentPositionInMillimeters()-10);    
    while (abs(y1_driver.getDistanceToTargetSigned())>0) {}
    y_endstop_triggered = false;

    y1_driver.setTargetPositionInMillimeters(1000);

    while (!y_endstop_triggered) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    } 
    y1_driver.emergencyStop();
    already_stopped_y = true;
    y1_driver.setCurrentPositionInMillimeters(0.0);
    y1_driver.setCurrentPositionInSteps(0);
    y1_driver.setCurrentPositionInRevolutions(0.0);
    y1_driver.setCurrentPositionAsHomeAndStop();
    state_msg.position.data[1] = 0.0;
    if (!pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
    y1_motor_asleep = true;
    status_msg.status = sb_msgs__msg__HivepokerStatus__IDLE;
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Y1 axis reached zero");

}



void homing_x() {

    status_msg.status = sb_msgs__msg__HivepokerStatus__MOVING;
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Homing X1 axis");
    if (!pcf8575.digitalWrite(X1_SLEEP_PIN, HIGH)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
    x1_motor_asleep = false;
    x1_driver.setTargetPositionInMillimeters(x1_driver.getCurrentPositionInMillimeters()-10);
    while (abs(x1_driver.getDistanceToTargetSigned())>0) {}
    left_endstop_triggered = false;

    x1_driver.setTargetPositionInMillimeters(1000);

    while (!left_endstop_triggered) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    } 
    x1_driver.emergencyStop();
    already_stopped_x = true;
    x1_driver.setCurrentPositionInMillimeters(0.0);
    x1_driver.setCurrentPositionInSteps(0);
    x1_driver.setCurrentPositionInRevolutions(0.0);
    x1_driver.setCurrentPositionAsHomeAndStop();
    state_msg.position.data[0] = 0.0;

    if (!pcf8575.digitalWrite(X1_SLEEP_PIN, LOW)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
    x1_motor_asleep = true;
    status_msg.status = sb_msgs__msg__HivepokerStatus__IDLE;
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "X1 axis reached zero");

}



void homing_arm1() {

  delay(500);
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Performing homing procedure");
  
  homing_y();
  homing_x();

  
  
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Homing procedure completed!");

}



//MICROROS FUNCTIONS ===============================================================================================================================================

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING,  \
        "Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); status_msg.error = sb_msgs__msg__HivepokerStatus__MICROROS_WARNING;}}

// --------------------------------------------------------------------------------------------------------------------------------
void error_loop(rcl_ret_t err){
  while(1){
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR,  \
        "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)err);
    status_msg.error = sb_msgs__msg__HivepokerStatus__MICROROS_ERROR;
    RCSOFTCHECK(rcl_publish(&status_pub, &status_msg, NULL));
    delay(1000);
  }
}

// --------------------------------------------------------------------------------------------------------------------------------


void move_callback(const void * msgin){
  // Cast messages to expected types
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  
  float x = (msg->position.data[0] + whycode_offset_x) * 1000 ;
  float y = (msg->position.data[1] + whycode_offset_y) * 1000 ;

  float acceleration_x = msg->effort.data[0] * 1000;
  float acceleration_y = msg->effort.data[1] * 1000;

  x1_driver.setAccelerationInMillimetersPerSecondPerSecond(acceleration_x);
  y1_driver.setAccelerationInMillimetersPerSecondPerSecond(acceleration_y);
  x1_driver.setDecelerationInMillimetersPerSecondPerSecond(acceleration_x);
  y1_driver.setDecelerationInMillimetersPerSecondPerSecond(acceleration_y);

  x1_driver.setSpeedInMillimetersPerSecond(msg->velocity.data[0] * 1000);
  y1_driver.setSpeedInMillimetersPerSecond(msg->velocity.data[1] * 1000);

  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Received Move Request: x=%.2f, y=%.2f", x, y);


  if (-y1_driver.getCurrentPositionInMillimeters() >= 1.0) {

    if (!((abs(-x1_driver.getCurrentPositionInMillimeters()-x)) < 5.0)) {
      homing_y();
    }

  }


  if (msg->position.data[0] == -1.0) {

    homing_x();

  } else {

    bool x_success = move_x_arm1(x);

  }


  if (msg->position.data[1] == -1.0) {

    homing_y();

  } else {

    bool y_success = move_y_arm1(y);

  }
  
}

// --------------------------------------------------------------------------------------------------------------------------------

void vicon_callback(const void * msgin) {

  const geometry_msgs__msg__PoseStamped * msg = (const geometry_msgs__msg__PoseStamped *) msgin;

  // if (!first_whycode_message) {

  float current_x = -x1_driver.getCurrentPositionInMillimeters() / 1000;
  float current_y = -y1_driver.getCurrentPositionInMillimeters() / 1000;

  float whycode_x = msg->pose.position.x;
  float whycode_y = msg->pose.position.y;

  whycode_offset_x = current_x - whycode_x;
  whycode_offset_y = current_y - whycode_y;

  // first_whycode_message = false;

  // }

  // float x = msg->pose.position.x / 1000;
  // float y = msg->pose.position.y / 1000;

  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Received Whycode position. The X offset is %.4f", whycode_offset_x);
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Received Whycode position. The Y offset is %.4f", whycode_offset_y);


  // x1_driver.setCurrentPositionInMillimeters(- (whycode_offset_x + x/1000));
  // y1_driver.setCurrentPositionInMillimeters(-y/1000);

}


// --------------------------------------------------------------------------------------------------------------------------------

void publishState(rcl_timer_t *timer, int64_t last_call_time) { 

  if (timer != NULL) {

    rcl_time_point_value_t now_ns;
    RCSOFTCHECK(rcl_clock_get_now(&sys_clock, &now_ns));

    state_msg.header.stamp.sec = now_ns / 1000000000ULL;
    state_msg.header.stamp.nanosec = now_ns % 1000000000ULL;

    state_msg.position.data[0] = -(x1_driver.getCurrentPositionInMillimeters() / 1000 + whycode_offset_x);
    state_msg.position.data[1] = -(y1_driver.getCurrentPositionInMillimeters() / 1000 + whycode_offset_y);
    state_msg.velocity.data[0] = -x1_driver.getCurrentVelocityInMillimetersPerSecond() / 1000;
    state_msg.velocity.data[1] = -y1_driver.getCurrentVelocityInMillimetersPerSecond() / 1000;
    RCSOFTCHECK(rcl_publish(&state_publisher, &state_msg, NULL));

    status_msg.header.stamp.sec = now_ns / 1000000000ULL;
    status_msg.header.stamp.nanosec = now_ns % 1000000000ULL;
    status_msg.left_endstop_triggered = left_endstop_triggered;
    status_msg.right_endstop_triggered = right_endstop_triggered;
    status_msg.y1_endstop_triggered = y_endstop_triggered;
    status_msg.y2_endstop_triggered = false;
    status_msg.middle_endstop_triggered = false;

    status_msg.x1_motor_asleep = x1_motor_asleep;
    status_msg.y1_motor_asleep = y1_motor_asleep;
    status_msg.x2_motor_asleep = x2_motor_asleep;
    status_msg.y2_motor_asleep = y2_motor_asleep;

    status_msg.x1_driver_fault = x1_driver_fault;
    status_msg.y1_driver_fault = y1_driver_fault;
    status_msg.x2_driver_fault = x2_driver_fault;
    status_msg.y2_driver_fault = y2_driver_fault;

    if (x1_driver_fault || y1_driver_fault || x2_driver_fault || y2_driver_fault) {
      status_msg.error = sb_msgs__msg__HivepokerStatus__DRIVER_FAULT;
    }

    RCSOFTCHECK(rcl_publish(&status_pub, &status_msg, NULL));


  }  
}


void healthCheck(rcl_timer_t *timer, int64_t last_call_time) {

  if (timer != NULL) {

    x1_driver_fault = (digitalRead(X1_FAULT_PIN) == HIGH);
    y1_driver_fault = (digitalRead(Y1_FAULT_PIN) == HIGH);
    x2_driver_fault = (digitalRead(X2_FAULT_PIN) == HIGH);
    y2_driver_fault = (digitalRead(Y2_FAULT_PIN) == HIGH);

  }
  
}


// --------------------------------------------------------------------------------------------------------------------------------


void logCallback(hivepoker::Logger::LogLevel level, const std::string& message) {    


    std::string prefix = hivepoker::Logger::getLogLevelPrefix(level);
    std::string msg_string = prefix + message;
    sprintf(log_msg.data.data, msg_string.c_str());
    log_msg.data.size = strlen(log_msg.data.data);

    RCSOFTCHECK(rcl_publish(&log_pub, &log_msg, NULL));


}



//LEFT ENDSTOP FUNCTIONS ===============================================================================================================================================
void IRAM_ATTR left_endstop_isr() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (xLeftEndstopTaskHandle != NULL) {
    vTaskNotifyGiveFromISR(xLeftEndstopTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}


void IRAM_ATTR right_endstop_isr() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (xRightEndstopTaskHandle != NULL) {
    vTaskNotifyGiveFromISR(xRightEndstopTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}


void IRAM_ATTR y_endstop_isr() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (xYEndstopTaskHandle != NULL) {
    vTaskNotifyGiveFromISR(xYEndstopTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}




void left_endstop_task(void *param) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for interrupt
    left_endstop_triggered = true;
  }
}


void right_endstop_task(void *param) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for interrupt
    right_endstop_triggered = true;
  }
}

void y_endstop_task(void *param) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for interrupt
    y_endstop_triggered = true;
  }
}





//SETUP ===============================================================================================================================================


void setup() {

  //TRANSPORT SETUP ##################################################################################################################

  Serial.begin(115200);
  set_microros_serial_transports(Serial);  
  delay(100);
  //Wait for the agent
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 10)) {
    delay(100);
  }  

  //NODE SETUP ##################################################################################################################
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "hivepoker", "", &support));
  RCCHECK(rclc_executor_init(&executor, &support.context, 15, &allocator));
  RCCHECK(rcl_clock_init(RCL_SYSTEM_TIME, &sys_clock, &allocator));

  //LOGGER SETUP ##################################################################################################################
  log_msg.data.data = (char * ) malloc(STR_SIZE * sizeof(char));
  log_msg.data.size = 0;
  log_msg.data.capacity = STR_SIZE;
  hivepoker::Logger::setLogLevel(hivepoker::Logger::LogLevel::DEBUG);
  hivepoker::Logger::setLogCallback(logCallback);
  RCCHECK(rclc_publisher_init_default(
    &log_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    LOGGER_TOPIC
  ));
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Initialized logger");

  // I2C EXTENDER SETUP ##################################################################################################################
  Wire.begin(SDA_PIN, SCL_PIN);
  bool connection_ok = pcf8575.begin(PCF8575_ADDR, &Wire);
  if (connection_ok) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Initialized I2C Extender");
  } else {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Can't connect to I2C Extender.");
    status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;
    while (1);
  }
  for (int i = 0; i < 16; i++) {
    pcf8575.pinMode(i, OUTPUT); // Set all pins as OUTPUT
  }
  if (!pcf8575.digitalWriteWord(0x0000))  {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}// Set all pins LOW
  
  //STATE PUBLISHER SETUP ##################################################################################################################
  sensor_msgs__msg__JointState__init(&state_msg);
  rosidl_runtime_c__double__Sequence__init(&state_msg.position, 4);
  rosidl_runtime_c__double__Sequence__init(&state_msg.velocity, 4);
  rosidl_runtime_c__double__Sequence__init(&state_msg.effort, 4);
  rosidl_runtime_c__String__Sequence__init(&state_msg.name, 4);
  rosidl_runtime_c__String__assign(&state_msg.name.data[0], "arm1_x");
  rosidl_runtime_c__String__assign(&state_msg.name.data[1], "arm1_y");
  rosidl_runtime_c__String__assign(&state_msg.name.data[2], "arm2_x");
  rosidl_runtime_c__String__assign(&state_msg.name.data[3], "arm2_y");
  for (int i=0; i<4; i++) {
    state_msg.position.data[i] = 0.0;
    state_msg.velocity.data[i] = 0.0;
    state_msg.effort.data[i] = 0.0;
  }
  RCCHECK(rclc_publisher_init_default(
    &state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    JOINT_STATE_TOPIC
  ));
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Initialized state publisher");


  //STATUS PUBLISHER SETUP ##################################################################################################################
  sb_msgs__msg__HivepokerStatus__init(&status_msg);
  status_msg.error = sb_msgs__msg__HivepokerStatus__OKAY;
  RCCHECK(rclc_publisher_init_default(
    &status_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sb_msgs, msg, HivepokerStatus),
    STATUS_TOPIC
  ));
  RCCHECK(rclc_timer_init_default(
      &state_timer,
      &support,
      RCL_MS_TO_NS(state_timer_timeout),
      publishState
  ));
  RCCHECK(rclc_executor_add_timer(&executor, &state_timer));
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Initialized statuss publisher");


  //HEALTH CHECK TIMER SETUP ##################################################################################################################
  RCCHECK(rclc_timer_init_default(
      &health_timer,
      &support,
      RCL_MS_TO_NS(health_timer_timeout),
      healthCheck
  ));
  RCCHECK(rclc_executor_add_timer(&executor, &health_timer));
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Initialized health check callback");


  //TARGET POSITION SUBSCRIBER SETUP ##################################################################################################################
  sensor_msgs__msg__JointState__init(&target_msg);
  rosidl_runtime_c__double__Sequence__init(&target_msg.position, 4);
  rosidl_runtime_c__double__Sequence__init(&target_msg.velocity, 4);
  rosidl_runtime_c__double__Sequence__init(&target_msg.effort, 4);
  rosidl_runtime_c__String__Sequence__init(&target_msg.name, 4);
  rosidl_runtime_c__String__assign(&target_msg.name.data[0], "arm1_x");
  rosidl_runtime_c__String__assign(&target_msg.name.data[1], "arm1_y");
  rosidl_runtime_c__String__assign(&target_msg.name.data[2], "arm2_x");
  rosidl_runtime_c__String__assign(&target_msg.name.data[3], "arm2_y");
  for (int i=0; i<4; i++) {
    target_msg.position.data[i] = 0.0;
    target_msg.velocity.data[i] = 0.0;
    target_msg.effort.data[i] = 0.0;
  }
  RCCHECK(rclc_subscription_init_default(
    &target_position_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    TARGET_POSITION_TOPIC
  ));
  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &target_position_sub, 
    &target_msg,
    &move_callback, 
    ON_NEW_DATA
  ));
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Initialized target position subscriber");


  //VICON POSITION SUBSCRIBER SETUP ##################################################################################################################
  geometry_msgs__msg__PoseStamped__init(&vicon_msg);
  RCCHECK(rclc_subscription_init_default(
    &vicon_position_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
    VICON_POSITION_TOPIC
  ));
  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &vicon_position_sub, 
    &vicon_msg,
    &vicon_callback, 
    ON_NEW_DATA
  ));
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Initialized target position subscriber");

  //LEFT ENDSTOP SETUP ##################################################################################################################
  pinMode(LEFT_ENDSTOP_PIN, INPUT_PULLDOWN);
  xTaskCreatePinnedToCore(left_endstop_task, "LeftEndstopTask", 4096, NULL, 1, &xLeftEndstopTaskHandle, 1);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENDSTOP_PIN), left_endstop_isr, FALLING);

  pinMode(RIGHT_ENDSTOP_PIN, INPUT_PULLDOWN);
  xTaskCreatePinnedToCore(right_endstop_task, "RightEndstopTask", 4096, NULL, 1, &xRightEndstopTaskHandle, 1);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENDSTOP_PIN), right_endstop_isr, FALLING);

  pinMode(MIDDLE_ENDSTOP_PIN, INPUT_PULLDOWN);
  xTaskCreatePinnedToCore(y_endstop_task, "YEndstopTask", 4096, NULL, 1, &xYEndstopTaskHandle, 1);
  attachInterrupt(digitalPinToInterrupt(MIDDLE_ENDSTOP_PIN), y_endstop_isr, FALLING);
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Added Endstops Interrupt");



  //X1 MOTOR SETUP ##################################################################################################################
  pinMode(X1_FAULT_PIN, INPUT_PULLUP);

  if (!pcf8575.digitalWrite(X1_RST_PIN, HIGH)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
  if (!pcf8575.digitalWrite(X1_SLEEP_PIN, LOW)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
  x1_motor_asleep = true;

  x1_driver.connectToPins(X1_STEP_PIN, X1_DIR_PIN);
  x1_driver.setStepsPerMillimeter(X_STEPS_PER_REVOLUTION / X_MM_TRAVELED_PER_REVOLUTION);
  x1_driver.setSpeedInMillimetersPerSecond(X_MAX_SPEED);
  x1_driver.setAccelerationInMillimetersPerSecondPerSecond(X_ACCELERATION);
  x1_driver.setDecelerationInMillimetersPerSecondPerSecond(X_ACCELERATION);
  x1_driver.setDirectionToHome(-1);
  x1_driver.startAsService(0);
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "X1 Motor initialized");

  //Y1 MOTOR SETUP ##################################################################################################################
  pinMode(Y1_FAULT_PIN, INPUT_PULLUP);

  if (!pcf8575.digitalWrite(Y1_RST_PIN, HIGH)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
  if (!pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW)) {status_msg.error = sb_msgs__msg__HivepokerStatus__I2C_ERROR;}
  y1_motor_asleep = true;

  y1_driver.connectToPins(Y1_STEP_PIN, Y1_DIR_PIN);
  y1_driver.setStepsPerMillimeter(Y_STEPS_PER_REVOLUTION / Y_MM_TRAVELED_PER_REVOLUTION);
  y1_driver.setSpeedInMillimetersPerSecond(Y_MAX_SPEED);
  y1_driver.setAccelerationInMillimetersPerSecondPerSecond(Y_ACCELERATION);
  y1_driver.setDecelerationInMillimetersPerSecondPerSecond(Y_ACCELERATION);
  y1_driver.startAsService(0);
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Y1 Motor initialized");


  homing_arm1();
  delay(500);


  ready = true;
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "System Initialized");


}




//LOOP ===============================================================================================================================================

void loop() {

  if (ready) {
        RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  } else {
      // Still initializing or waiting for a trigger
  }

}



