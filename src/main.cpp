#include <Arduino.h>

//MICROROS
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcutils/logging.h>
#include <rclc/executor.h>
#include <rcl/time.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
//INTERFACES
#include <std_msgs/msg/string.h>
#include <hivepoker_interfaces/msg/arms_positions.h>
#include <hivepoker_interfaces/msg/state.h>
#include <hivepoker_interfaces/srv/move.h>
#include <hivepoker_interfaces/srv/trajectory.h>
#include <hivepoker_interfaces/srv/stop.h>
//I2C EXTENDER
#include <Wire.h>
#include <Adafruit_PCF8575.h>
//SEPPERS
#include <ESP_FlexyStepper.h>
//MISC
#include <pthread.h>
#include <logger.h>
//HARDWARE
#include <new_pin_definitions.h>
#include <parameters.h>


#define STR_SIZE 500


//STEPPER DEFINITIONS ===============================================================================================================================================
ESP_FlexyStepper x1_driver;
ESP_FlexyStepper y1_driver;


// I2C DEFINITIONS ===============================================================================================================================================
Adafruit_PCF8575 pcf8575;

//MICROROS DEFINITIONS ===============================================================================================================================================
hivepoker_interfaces__srv__Move_Request request_msg_move;
hivepoker_interfaces__srv__Move_Response response_msg_move;
hivepoker_interfaces__srv__Stop_Request request_msg_stop;
hivepoker_interfaces__srv__Stop_Response response_msg_stop;
hivepoker_interfaces__srv__Trajectory_Request* request_msg_trajectory;
hivepoker_interfaces__srv__Trajectory_Response* response_msg_trajectory;
hivepoker_interfaces__msg__ArmsPositions state;
rcl_publisher_t state_publisher, log_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_service_t move_srv, stop_srv, trajectory_srv;
rcl_timer_t timer;
rcl_clock_t sys_clock;
const unsigned int timer_timeout = 500; //ms
std_msgs__msg__String log_msg;
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





//MOVING FUNCTIONS ===============================================================================================================================================
bool move_x_arm1(float x) {

  pcf8575.digitalWrite(X1_SLEEP_PIN, HIGH);

  if (x<=0 && left_endstop_triggered) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Left movement not allowed");
    pcf8575.digitalWrite(X1_SLEEP_PIN, LOW);
    return false;
  }

  if (x>=420.0 && right_endstop_triggered) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Right movement not allowed");
    pcf8575.digitalWrite(X1_SLEEP_PIN, LOW);
    return false;
  }

  x1_driver.setTargetPositionInMillimeters(-x);
  state.arm1.state = hivepoker_interfaces__msg__State__MOVING;

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

    if (state.arm1.pos_x > 3.0 && left_endstop_triggered) {left_endstop_triggered = false; already_stopped_x = false;}
    if (state.arm1.pos_x < 420.0 && right_endstop_triggered) {right_endstop_triggered = false; already_stopped_x = false;}

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  } while (
    abs(x1_driver.getDistanceToTargetSigned()) > 0.0
  );

  pcf8575.digitalWrite(X1_SLEEP_PIN, LOW);
  return true;

}





bool move_y_arm1(float y) {

  pcf8575.digitalWrite(Y1_SLEEP_PIN, HIGH);


  if (y<Y_MIN_VALUE_MM) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Down movement not allowed");
    pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);
    return false;
  }

  if (y>=Y_MAX_VALUE_MM) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Up movement not allowed");
    pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);
    return false;
  }

  y1_driver.setTargetPositionInMillimeters(-y);
  state.arm1.state = hivepoker_interfaces__msg__State__MOVING;

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


    if (state.arm1.pos_y > 3.0 && y_endstop_triggered) {y_endstop_triggered = false; already_stopped_y = false;}
    

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  } while (
    abs(y1_driver.getDistanceToTargetSigned()) > 0.0
  );

  pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);
  return true;

}




void homing_y() {

    pcf8575.digitalWrite(Y1_SLEEP_PIN, HIGH);
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
    state.arm1.pos_y = 0.0;
    pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Y1 axis reached zero");

}



void homing_x() {

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Homing X1 axis");
    pcf8575.digitalWrite(X1_SLEEP_PIN, HIGH);
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
    state.arm1.pos_x = 0.0;

    pcf8575.digitalWrite(X1_SLEEP_PIN, LOW);
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "X1 axis reached zero");

}



void homing_arm1() {

  delay(500);
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Performing homing procedure");
  
  homing_y();
  homing_x();

  
  
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Homing procedure completed!");

}



bool validate_trajectory(float x_rel, float y_rel) {

  float x = start_x + x_rel;
  float y = start_y + y_rel;

  // pcf8575.digitalWrite(X1_SLEEP_PIN, HIGH);
  // pcf8575.digitalWrite(Y1_SLEEP_PIN, HIGH);

  if (y<Y_MIN_VALUE_MM) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Trajectory too low");
    pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);
    return false;
  }

  if (y>=Y_MAX_VALUE_MM) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Trajectory too high");
    pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);
    return false;
  }

  if (x<=0) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Trajectory too much to the left");
    pcf8575.digitalWrite(X1_SLEEP_PIN, LOW);
    return false;
  }

  if (x>=420.0) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Trajectory too much to the right");
    pcf8575.digitalWrite(X1_SLEEP_PIN, LOW);
    return false;
  }


  // state.arm1.state = hivepoker_interfaces__msg__State__MOVING;

  // x1_driver.setTargetPositionInMillimeters(-x);

  // do {
  //   // x1_driver.processMovement();
  //   // y1_driver.processMovement();
  //   // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  // } while (
  //   (abs(x1_driver.getDistanceToTargetSigned()) > 30.0)
  // );

  // y1_driver.setTargetPositionInMillimeters(-y);

  // do {
  //   // x1_driver.processMovement();
  //   // y1_driver.processMovement();
  //   // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  // } while (
  //   (abs(y1_driver.getDistanceToTargetSigned()) > 30.0)
  // );

  // pcf8575.digitalWrite(X1_SLEEP_PIN, LOW);
  // pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);

  return true;

}





//MICROROS FUNCTIONS ===============================================================================================================================================

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING,  \
        "Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);}}

// --------------------------------------------------------------------------------------------------------------------------------
void error_loop(rcl_ret_t err){
  while(1){
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR,  \
        "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)err);
    delay(1000);
  }
}

// --------------------------------------------------------------------------------------------------------------------------------


void move_callback(const void * request_msg, void * response_msg){
  // Cast messages to expected types
  const hivepoker_interfaces__srv__Move_Request * req_in = (const hivepoker_interfaces__srv__Move_Request *)request_msg;
  hivepoker_interfaces__srv__Move_Response * res_in = (hivepoker_interfaces__srv__Move_Response *)response_msg;

  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Received Move Request: x=%.2f, y=%.2f", req_in->x, req_in->y);

  
  float x = req_in->x;
  float y = req_in->y;

  if (req_in->arm == hivepoker_interfaces__srv__Move_Response__ARM1) {

    if (-y1_driver.getCurrentPositionInMillimeters() >= 1.0) {

      hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Received Move Request: x=%.2f", abs(-x1_driver.getCurrentPositionInMillimeters()-x));
      if (!((abs(-x1_driver.getCurrentPositionInMillimeters()-x)) < 5.0)) {
        homing_y();
      }
    }


    bool x_success = move_x_arm1(x);
    if (!x_success) {
      res_in->success = false;
    } else {
      res_in->success = true;
    }


    bool y_success = move_y_arm1(y);

    if (!y_success) {
      res_in->success = false;
    } else {
      res_in->success = true;
    }
    
  }  


}

// --------------------------------------------------------------------------------------------------------------------------------



void trajectory_callback(const void * request_msg, void * response_msg){
  // Cast messages to expected types
  const hivepoker_interfaces__srv__Trajectory_Request * req_in = (const hivepoker_interfaces__srv__Trajectory_Request *)request_msg;
  hivepoker_interfaces__srv__Trajectory_Response * res_in = (hivepoker_interfaces__srv__Trajectory_Response *)response_msg;

  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Received Trajectory");

  // x1_driver.stopService();
  // y1_driver.stopService();
  // x1_driver.startAsService(1);
  // y1_driver.startAsService(1);

//   x1_driver.setSpeedInMillimetersPerSecond(50.0);
//   y1_driver.setSpeedInMillimetersPerSecond(50.0);

  if (req_in->x.size != req_in->y.size) {
      res_in->success = false;
      hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Received Bad Trajectory, ignoring..");
      return;
  }

  bool errors = false;

  pcf8575.digitalWrite(X1_SLEEP_PIN, HIGH);
  pcf8575.digitalWrite(Y1_SLEEP_PIN, HIGH);

  if (req_in->arm == hivepoker_interfaces__srv__Trajectory_Response__ARM1) {

    start_x = -x1_driver.getCurrentPositionInMillimeters();
    start_y = -y1_driver.getCurrentPositionInMillimeters();

    for (size_t i = 0; i < req_in->x.size; i++) {
      // hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Moving to point %d x=%.2f, y=%.2f",i,  req_in->x.data[i], req_in->y.data[i]);
        bool success = validate_trajectory(req_in->x.data[i], req_in->y.data[i]);
        if (!success) {
          hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Trajecctory points not feasible, ignoring..");
          res_in->success = false;
          break;
        }
    }

    for (size_t i = 0; i < req_in->x.size; i++) {

      float x = start_x + req_in->x.data[i];
      float y = start_y + req_in->y.data[i];
      x1_driver.setTargetPositionInMillimeters(-x);

      // do {
      // } while (
      //   (abs(x1_driver.getDistanceToTargetSigned()) > 10.0)
      // );

      y1_driver.setTargetPositionInMillimeters(-y);

      // do {
      // } while (
      //   (abs(y1_driver.getDistanceToTargetSigned()) > 10.0)
      // );
      delay(5);

    }

 


    x1_driver.setSpeedInMillimetersPerSecond(X_MAX_SPEED);
    y1_driver.setSpeedInMillimetersPerSecond(Y_MAX_SPEED);
    pcf8575.digitalWrite(X1_SLEEP_PIN, LOW);
    pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);

    // x1_driver.startAsService(0);
    // y1_driver.startAsService(0);

  }


  res_in->success = true;

}

// --------------------------------------------------------------------------------------------------------------------------------

void stop_callback(const void * request_msg, void * response_msg){
  // Cast messages to expected types
  const hivepoker_interfaces__srv__Stop_Request * req_in = (const hivepoker_interfaces__srv__Stop_Request *)request_msg;
  hivepoker_interfaces__srv__Stop_Response * res_in = (hivepoker_interfaces__srv__Stop_Response *)response_msg;

  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Received Stop Request");
  x1_driver.emergencyStop();
  y1_driver.emergencyStop();

  homing_arm1();

  res_in->success = true;

}

// --------------------------------------------------------------------------------------------------------------------------------

void publishState(rcl_timer_t *timer, int64_t last_call_time) { 

  if (timer != NULL) {

    rcl_time_point_value_t now_ns;
    RCSOFTCHECK(rcl_clock_get_now(&sys_clock, &now_ns));

    state.header.stamp.sec = now_ns / 1000000000ULL;
    state.header.stamp.nanosec = now_ns % 1000000000ULL;

    state.arm1.pos_x = -x1_driver.getCurrentPositionInMillimeters();
    state.arm1.pos_y = -y1_driver.getCurrentPositionInMillimeters();
    state.arm1.vel_x = -x1_driver.getCurrentVelocityInMillimetersPerSecond();
    state.arm1.vel_y = -y1_driver.getCurrentVelocityInMillimetersPerSecond();
    RCSOFTCHECK(rcl_publish(&state_publisher, &state, NULL));

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
    "logger"
  ));
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Initialized logger");

  // I2C EXTENDER SETUP ##################################################################################################################
  Wire.begin(SDA_PIN, SCL_PIN);
  bool connection_ok = pcf8575.begin(PCF8575_ADDR, &Wire);
  if (connection_ok) {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Initialized I2C Extender");
  } else {
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Can't connect to I2C Extender.");
    while (1);
  }
  for (int i = 0; i < 16; i++) {
    pcf8575.pinMode(i, OUTPUT); // Set all pins as OUTPUT
  }
  pcf8575.digitalWriteWord(0x0000); // Set all pins LOW
  
  //STATE PUBLISHER SETUP ##################################################################################################################
  hivepoker_interfaces__msg__ArmsPositions__init(&state);
  state.header.frame_id.data = NULL;
  state.header.frame_id.capacity = 0;
  state.header.frame_id.size = 0;
  RCCHECK(rclc_publisher_init_default(
    &state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(hivepoker_interfaces, msg, ArmsPositions),
    "state"
  ));
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      publishState
  ));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Initialized state publisher");

  //MOVE SERVICE SETUP ##################################################################################################################
  RCCHECK(rclc_service_init_default(
          &move_srv, 
          &node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(hivepoker_interfaces, srv, Move), 
          "move"
  ));
  RCCHECK(rclc_executor_add_service(
          &executor, 
          &move_srv, 
          &request_msg_move,
          &response_msg_move, 
          move_callback
  ));
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Added Move Service");

  //STOP SERVICE SETUP ##################################################################################################################
  RCCHECK(rclc_service_init_default(
          &stop_srv, 
          &node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(hivepoker_interfaces, srv, Stop), 
          "stop"
  ));
  RCCHECK(rclc_executor_add_service(
          &executor, 
          &stop_srv, 
          &request_msg_stop,
          &response_msg_stop, 
          stop_callback
  ));
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Added Stop Service");

  //TRAJECTORY SERVICE SETUP ##################################################################################################################
  request_msg_trajectory = hivepoker_interfaces__srv__Trajectory_Request__create();
  response_msg_trajectory = hivepoker_interfaces__srv__Trajectory_Response__create();
  rosidl_runtime_c__double__Sequence__init(&request_msg_trajectory->x, 300);
  rosidl_runtime_c__double__Sequence__init(&request_msg_trajectory->y, 300);
  RCCHECK(rclc_service_init_default(
          &trajectory_srv, 
          &node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(hivepoker_interfaces, srv, Trajectory), 
          "trajectory"
  ));
  RCCHECK(rclc_executor_add_service(
          &executor, 
          &trajectory_srv, 
          request_msg_trajectory,
          response_msg_trajectory, 
          trajectory_callback
  ));
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "Added Trajectory Service");


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
  // pinMode(X1_RST_PIN, OUTPUT);
  pinMode(X1_FAULT_PIN, INPUT);
  // pinMode(X1_SLEEP_PIN, OUTPUT);

  pcf8575.digitalWrite(X1_RST_PIN, HIGH);
  pcf8575.digitalWrite(X1_SLEEP_PIN, LOW);

  x1_driver.connectToPins(X1_STEP_PIN, X1_DIR_PIN);
  x1_driver.setStepsPerMillimeter(X_STEPS_PER_REVOLUTION / X_MM_TRAVELED_PER_REVOLUTION);
  x1_driver.setSpeedInMillimetersPerSecond(X_MAX_SPEED);
  x1_driver.setAccelerationInMillimetersPerSecondPerSecond(X_ACCELERATION);
  x1_driver.setDecelerationInMillimetersPerSecondPerSecond(X_ACCELERATION);
  x1_driver.setDirectionToHome(-1);
  x1_driver.startAsService(0);
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "X1 Motor initialized");

  //Y1 MOTOR SETUP ##################################################################################################################
  // pinMode(Y1_RST_PIN, OUTPUT);
  pinMode(Y1_FAULT_PIN, INPUT);
  // pinMode(Y1_SLEEP_PIN, OUTPUT);

  pcf8575.digitalWrite(Y1_RST_PIN, HIGH);
  pcf8575.digitalWrite(Y1_SLEEP_PIN, LOW);

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

  
  // x1_driver.setSpeedInMillimetersPerSecond(X_MAX_SPEED);
  // y1_driver.setSpeedInMillimetersPerSecond(Y_MAX_SPEED);

  // if (left_endstop_triggered || right_endstop_triggered) {
  //   hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING, "Endstop Triggered");
  // }

}



