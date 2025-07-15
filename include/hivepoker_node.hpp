#ifndef HIVEPOKER_NODE
#define HIVEPOKER_NODE
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcutils/logging.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <logger.h>

#include <hivepoker_actions/action/move_arm.h>
#include <hivepoker_controller.hpp>



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){hivepoker::HivepokerNode::error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING,  \
        "Failed status: %d. Continuing", (int)temp_rc);}}


#define STR_SIZE 500
#define GOALS_TIMEOUT_MS 1000 


namespace hivepoker
{
    
class HivepokerNode {

    public:

        HivepokerNode();

        void setup();

        static void error_loop(rcl_ret_t error);
        void logCallback(hivepoker::Logger::LogLevel level, const std::string& message);

        static void move_arm1_action_callback_wrapper(rcl_timer_t *timer, int64_t last_call_time);
        void move_arm1_action_callback(rcl_timer_t * timer, int64_t last_call_time);

        // Static wrappers
        static rcl_ret_t handle_move_arm1_goal_wrapper(rclc_action_goal_handle_t * goal_handle, void * context);
        static bool handle_move_arm1_cancel_wrapper(rclc_action_goal_handle_t * goal_handle, void * context);
        

        rcl_ret_t handle_move_arm1_goal(rclc_action_goal_handle_t * goal_handle, void * context);
        bool handle_move_arm1_cancel(rclc_action_goal_handle_t * goal_handle, void * context);


        void run();


    private:

        static HivepokerNode* self_instance;

        rcl_publisher_t log_pub;
        rclc_executor_t executor;
        rclc_support_t support;
        rcl_allocator_t allocator;
        rcl_node_t node;
        rcl_timer_t timer;

        rclc_action_server_t arm1_action_server;
        hivepoker_actions__action__MoveArm_SendGoal_Request  ros_goal_request[1];
        hivepoker_actions__action__MoveArm_GetResult_Response response;
        rclc_action_goal_handle_t *arm1_goal_handle = NULL;

        std_msgs__msg__String log_msg;

        HivepokerController controller;
        bool move_arm1_goal_active = false;


};






} // namespace hivepoker




#endif