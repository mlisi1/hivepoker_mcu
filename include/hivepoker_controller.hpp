#ifndef HIVEPOKER_CONTROLLER
#define HIVEPOKER_CONTROLLER
#include <logger.h>
#include <stepper_driver.hpp>
#include <pin_definitions.h>
#include <parameters.h>

namespace hivepoker
{


class HivepokerController {

    public:

        HivepokerController();

        void init(int arms_number = 1, bool inverted_endstop = false);


        void attachLeftEndstop();
        void attachRightEndstop();
        void attachMiddleEndstop();

        void MoveArm1(float x, float y);
        bool isArm1GoalReached();
        std::tuple<float, float> getArm1Position();

        void run();
        

    private:

        static void IRAM_ATTR isrWrapperLeft();
        static void IRAM_ATTR isrWrapperRight();
        static void IRAM_ATTR isrWrapperMiddle();

        void IRAM_ATTR handleInterruptLeft(); 
        void IRAM_ATTR handleInterruptRight(); 
        void IRAM_ATTR handleInterruptMiddle(); 


        static void taskWrapperLeft(void* param);
        static void taskWrapperRight(void* param);
        static void taskWrapperMiddle(void* param);

        void endstopTaskLeft();
        void endstopTaskRight();
        void endstopTaskMiddle();


        



    private:
        
        static HivepokerController* instance;
        int arms_number_ = 0;
        StepperDriver x1_driver, x2_driver, y1_driver, y2_driver;
        bool inverted_endstop_ = false;
        bool initialized = false;

        TaskHandle_t taskHandleL, taskHandleR, taskHandleM = nullptr;
        bool endstop_l, endstop_r, endstop_m = false;


};

    
} // namespace hivepoker

#endif