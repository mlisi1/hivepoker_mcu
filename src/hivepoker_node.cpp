#include <hivepoker_node.hpp>

hivepoker::HivepokerNode* hivepoker::HivepokerNode::self_instance = nullptr;


hivepoker::HivepokerNode::HivepokerNode() {
}


void hivepoker::HivepokerNode::setup() {

    // controller.init(1);
    // controller.attachLeftEndstop();
    // controller.attachRightEndstop();

    // Serial.begin(115200);
    // set_microros_serial_transports(Serial);

    self_instance = this;

    IPAddress ip(192, 168, 0, 103);
    uint16_t port = 8888;
    set_microros_wifi_transports((char*)"Tenda_30AE50", (char *)"CreativeLab2025?", ip, port);
    
    delay(500);

    allocator = rcl_get_default_allocator();

    log_msg.data.data = (char * ) malloc(STR_SIZE * sizeof(char));
    log_msg.data.size = 0;
    log_msg.data.capacity = STR_SIZE;

    hivepoker::Logger::setLogLevel(hivepoker::Logger::LogLevel::DEBUG);
    hivepoker::Logger::setLogCallback(std::bind(&HivepokerNode::logCallback, this, std::placeholders::_1, std::placeholders::_2));

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "hivepoker", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &log_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "logger"));

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "[HivepokerNode] - Initialized Logger");


    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 15, &allocator));
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "[HivepokerNode] - Initialized Executor");
    RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));


    RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(GOALS_TIMEOUT_MS),
            hivepoker::HivepokerNode::move_arm1_action_callback_wrapper)
    );
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "[HivepokerNode] - Added goal callback");

    RCCHECK(
      rclc_action_server_init_default(
        &arm1_action_server,
        &node,
        &support,
        ROSIDL_GET_ACTION_TYPE_SUPPORT(hivepoker_actions, MoveArm),
        "move_ee"
    ));

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "[HivepokerNode] - Initialized Action Server");
    RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));


    RCCHECK(
    rclc_executor_add_action_server(
      &executor,
      &arm1_action_server,
      1,
      ros_goal_request,
      sizeof(hivepoker_actions__action__MoveArm_SendGoal_Request),
      hivepoker::HivepokerNode::handle_move_arm1_goal_wrapper,
      hivepoker::HivepokerNode::handle_move_arm1_cancel_wrapper,
      (void *) &arm1_action_server)
    );

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "[HivepokerNode] - Added Action Server to Executor");
    RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "[HivepokerNode] - System Initialized");
    RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));

}





bool hivepoker::HivepokerNode::handle_move_arm1_cancel_wrapper(rclc_action_goal_handle_t *goal_handle, void *context) {
  hivepoker::HivepokerNode *self = hivepoker::HivepokerNode::self_instance;
  return self->handle_move_arm1_cancel(goal_handle, context);
}

rcl_ret_t hivepoker::HivepokerNode::handle_move_arm1_goal_wrapper(rclc_action_goal_handle_t * goal_handle, void * context) {

  hivepoker::HivepokerNode *self = hivepoker::HivepokerNode::self_instance;
  return self->handle_move_arm1_goal(goal_handle, context);

}

void hivepoker::HivepokerNode::move_arm1_action_callback_wrapper(rcl_timer_t *timer, int64_t last_call_time) {

  hivepoker::HivepokerNode *self = hivepoker::HivepokerNode::self_instance;
  self->move_arm1_action_callback(timer, last_call_time);

}




void hivepoker::HivepokerNode::move_arm1_action_callback(rcl_timer_t * timer, int64_t last_call_time) {

  (void) last_call_time;

  if (timer == NULL || !move_arm1_goal_active || arm1_goal_handle == NULL) {
        response.result.success = false;
        return;
  }

  rcl_action_goal_state_t goal_state = GOAL_STATE_SUCCEEDED;
  move_arm1_goal_active = false;

  hivepoker_actions__action__MoveArm_SendGoal_Request * req =
    (hivepoker_actions__action__MoveArm_SendGoal_Request *) arm1_goal_handle->ros_goal_request;

  response.result.success = true;




  rcl_ret_t rc;
  do {
    rc = rclc_action_send_result(arm1_goal_handle, goal_state, &response);
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "[HivepokerNode] - Trying to send result - %d", rc);
    RCSOFTCHECK(rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(10)));
    vTaskDelay(pdMS_TO_TICKS(100));
  } while (rc != RCL_RET_OK);
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "[HivepokerNode] - Action executed");
  arm1_goal_handle = NULL;
  return;


}




rcl_ret_t hivepoker::HivepokerNode::handle_move_arm1_goal(rclc_action_goal_handle_t * goal_handle, void * context) {

  (void) context;

  hivepoker_actions__action__MoveArm_SendGoal_Request * req =
    (hivepoker_actions__action__MoveArm_SendGoal_Request *) goal_handle->ros_goal_request;

  arm1_goal_handle = goal_handle;
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "[HivepokerNode] - Accepted action");

  move_arm1_goal_active = true;

  return RCL_RET_ACTION_GOAL_ACCEPTED;

}





bool hivepoker::HivepokerNode::handle_move_arm1_cancel(rclc_action_goal_handle_t * goal_handle, void * context) {

  (void) context;
  (void) goal_handle;
  hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING, "[HivepokerNode] - Action canceled");

  return true;
}




void hivepoker::HivepokerNode::run() {

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  // controller.run();

}


void hivepoker::HivepokerNode::error_loop(rcl_ret_t error) {

  while(1){
    // hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "Error Loop %s", rcl_get_error_string());
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR,  \
        "[HivepokerNode] - Failed status: %d. Aborting.", (int)error);
    delay(1000);
  }

}




void hivepoker::HivepokerNode::logCallback(hivepoker::Logger::LogLevel level, const std::string& message) {

  std::string prefix = hivepoker::Logger::getLogLevelPrefix(level);
  std::string msg_string = prefix + message;
  sprintf(log_msg.data.data, msg_string.c_str());
  log_msg.data.size = strlen(log_msg.data.data);

  RCSOFTCHECK(rcl_publish(&log_pub, &log_msg, NULL));

}