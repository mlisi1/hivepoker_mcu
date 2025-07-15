// #include <Arduino.h>

// #include <micro_ros_platformio.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rcutils/logging.h>
// #include <rclc/executor.h>

// #include <std_msgs/msg/string.h>

// #include <logger.h>

// // #include <example_interfaces/action/fibonacci.h>
// #include <hivepoker_actions/action/move_end_effector.h>


// #include <Stepper.h>
// #include <pthread.h>
// #include "esp_heap_caps.h"
// #include <math.h>


// const int stepsPerRevolution = 200;
// bool move_ee_goal_active = false;

// bool limit_triggered = false;

// #define STR_SIZE 500
// #define DRV833_IN1 12
// #define DRV833_IN2 14
// #define DRV833_IN3 27
// #define DRV833_IN4 26

// #define LIMIT 34


// volatile bool endstop_triggered = false;
// TaskHandle_t xEndstopTaskHandle = NULL;


// void IRAM_ATTR endstop_isr() {
//   BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//   if (xEndstopTaskHandle != NULL) {
//     vTaskNotifyGiveFromISR(xEndstopTaskHandle, &xHigherPriorityTaskWoken);
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//   }
// }




// void endstop_task(void *param) {
//   while (1) {
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for interrupt

//     limit_triggered = !limit_triggered;

//     hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING, "Endstop: %d", limit_triggered);

//   }
// }

// Stepper myStepper(stepsPerRevolution, DRV833_IN1, DRV833_IN2, DRV833_IN3, DRV833_IN4);


// rcl_publisher_t publisher, log_pub;
// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;

// rcl_timer_t timer;
// const unsigned int timer_timeout = 1000; //ms


// rclc_action_server_t action_server;
// hivepoker_actions__action__MoveEndEffector_SendGoal_Request  ros_goal_request[1];
// hivepoker_actions__action__MoveEndEffector_GetResult_Response response;
// rclc_action_goal_handle_t *g_goal_handle = NULL;

// std_msgs__msg__String log_msg;


// // #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
// // #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
// #define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING,  \
//         "Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);}}


// void error_loop(rcl_ret_t err){
//   while(1){
//     // hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Error Loop %s", rcl_get_error_string());
//     hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR,  \
//         "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)err);
//     delay(1000);
//   }
// }



// void logCallback(hivepoker::Logger::LogLevel level, const std::string& message) {    


//     std::string prefix = hivepoker::Logger::getLogLevelPrefix(level);
//     std::string msg_string = prefix + message;
//     sprintf(log_msg.data.data, msg_string.c_str());
//     log_msg.data.size = strlen(log_msg.data.data);

//     RCSOFTCHECK(rcl_publish(&log_pub, &log_msg, NULL));
//     // delay(10);

// }






// void * move_ee_worker(void * args)
// {
// // (void) args;
// rclc_action_goal_handle_t * goal_handle = (rclc_action_goal_handle_t *) args;
// rcl_action_goal_state_t goal_state = GOAL_STATE_SUCCEEDED;

// hivepoker_actions__action__MoveEndEffector_SendGoal_Request * req =
//   (hivepoker_actions__action__MoveEndEffector_SendGoal_Request *) goal_handle->ros_goal_request;

// hivepoker_actions__action__MoveEndEffector_GetResult_Response response;
// hivepoker_actions__action__MoveEndEffector_GetResult_Response feedback;

// hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Started worker thread");
// usleep(1e5);

// response.result.success = true;

// rcl_ret_t rc;
// do {
//   rc = rclc_action_send_result(goal_handle, goal_state, &response);
//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Trying to send result - %s", rc);
//   usleep(1e6);
// } while (rc != RCL_RET_OK);

// pthread_exit(NULL);
// }






// void goal_callback(rcl_timer_t * timer, int64_t last_call_time)
// {
// 	(void) last_call_time;

//   if (timer == NULL || !move_ee_goal_active || g_goal_handle == NULL) {
//         response.result.success = false;
//         return;
//   }

//   rcl_action_goal_state_t goal_state = GOAL_STATE_SUCCEEDED;

//   move_ee_goal_active = false;

//   hivepoker_actions__action__MoveEndEffector_SendGoal_Request * req =
//     (hivepoker_actions__action__MoveEndEffector_SendGoal_Request *) g_goal_handle->ros_goal_request;

//   response.result.success = true;
//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Requested position - X: %f, Y: %f", req->goal.ee_position[0], req->goal.ee_position[1]);
//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Executing action");
//   int steps = static_cast<int>(req->goal.ee_position[0]);
//   int s = (steps > 0) - (steps < 0);
//   int halved = static_cast<int>(steps/2);
//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Turning the motor for %d steps; sign: %d", steps, s);

//   for (int i=0; i<abs(halved); i++) {    

//     if (limit_triggered) {
//       goal_state = GOAL_STATE_ABORTED;
//       break;
//     }

//     myStepper.step(s * 2);

//   }
 
//   // myStepper.step(steps);
//   // if (req->goal.ee_position[0] >= 0.0) {
    
//   //   myStepper.step(400);
//   // } else {
//   //   myStepper.step(-400);
//   // }
//   // rclc_action_send_result(g_goal_handle, GOAL_STATE_SUCCEEDED, &response);
//   rcl_ret_t rc;
//   do {
//     rc = rclc_action_send_result(g_goal_handle, goal_state, &response);
//     hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Trying to send result - %d", rc);
//     RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));
//     vTaskDelay(pdMS_TO_TICKS(100));
//   } while (rc != RCL_RET_OK);
//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Action executed");
//   g_goal_handle = NULL;
//   return;
// }





// rcl_ret_t handle_goal(rclc_action_goal_handle_t * goal_handle, void * context)
// {

//   (void) context;

//   hivepoker_actions__action__MoveEndEffector_SendGoal_Request * req =
//     (hivepoker_actions__action__MoveEndEffector_SendGoal_Request *) goal_handle->ros_goal_request;

//   g_goal_handle = goal_handle;
//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Accepted action");

//   move_ee_goal_active = true;

//   return RCL_RET_ACTION_GOAL_ACCEPTED;
// }






// bool handle_cancel(rclc_action_goal_handle_t * goal_handle, void * context)
// {
//   (void) context;
//   (void) goal_handle;
//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING, "Action canceled");

//   return true;
// }






// void setup() {

//   // Serial.begin(115200);
//   // set_microros_serial_transports(Serial);

//   IPAddress ip(192, 168, 0, 103);
//   uint16_t port = 8888;
//   set_microros_wifi_transports("Tenda_30AE50", "CreativeLab2025?", ip, port);
  
//   delay(500);


  


//   myStepper.setSpeed(60); // rpm


//   allocator = rcl_get_default_allocator();

//   log_msg.data.data = (char * ) malloc(STR_SIZE * sizeof(char));
//   log_msg.data.size = 0;
//   log_msg.data.capacity = STR_SIZE;

//   hivepoker::Logger::setLogLevel(hivepoker::Logger::LogLevel::INFO);
//   hivepoker::Logger::setLogCallback(logCallback);

//   pinMode(LIMIT, INPUT_PULLDOWN);
  

//   //create init_options
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   // create node
//   RCCHECK(rclc_node_init_default(&node, "hivepoker", "", &support));

//   // create publisher
//   RCCHECK(rclc_publisher_init_default(
//     &log_pub,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
//     "logger"));

//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "====================================\nInitialized Logger");
//   // RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));

//   // create executor
//   RCCHECK(rclc_executor_init(&executor, &support.context, 15, &allocator));
//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Initialized Executor");
//   RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));


//   RCCHECK(rclc_timer_init_default(
// 		&timer,
// 		&support,
// 		RCL_MS_TO_NS(timer_timeout),
// 		goal_callback)
//   );
//   RCCHECK(rclc_executor_add_timer(&executor, &timer));
//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Added goal callback");

  
//   RCCHECK(
//     rclc_action_server_init_default(
//       &action_server,
//       &node,
//       &support,
//       ROSIDL_GET_ACTION_TYPE_SUPPORT(hivepoker_actions, MoveEndEffector),
//       "move_ee"
//   ));

//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Initialized Action Server");
//   RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));


//   RCCHECK(
//     rclc_executor_add_action_server(
//       &executor,
//       &action_server,
//       1,
//       ros_goal_request,
//       sizeof(hivepoker_actions__action__MoveEndEffector_SendGoal_Request),
//       handle_goal,
//       handle_cancel,
//       (void *) &action_server)
//     );

//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Added Action Server to Executor");
//   RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));

//   xTaskCreatePinnedToCore(endstop_task, "EndstopTask", 4096, NULL, 1, &xEndstopTaskHandle, 1);
//   attachInterrupt(digitalPinToInterrupt(LIMIT), endstop_isr, CHANGE);

//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Added Endstop Interrupt");
//   RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));

//   hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "System Initialized");
//   RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));

// }

// void loop() {

//   RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  
  
//   // delay(50);


// }




#include <hivepoker_node.hpp>

hivepoker::HivepokerNode node;




void setup() {

  node.setup();

}




void loop() {

  node.run();

}