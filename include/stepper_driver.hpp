#ifndef STEPPER_DRIVER
#define STEPPER_DRIVER
#include <AccelStepper.h>
#include <logger.h>


namespace hivepoker
{


class StepperDriver {

    public:

        StepperDriver();

        void init(int step_pin, int dir_pin, int reset_pin, int fault_pin, int sleep_pin, float stepsPerRev, float travelPerRev_mm, const char* name = "");

        void moveTo(double pos); //mm

        float getCurrentPosition();

        void run();

        void stopMotor();

        void checkMotorState();

        void recoveryProcedure();

        bool isMoving();

        void sleep();

        void wakeUp();

        bool isSleeping();


    private:

        float stepsPerRev_, travelPerRev_mm_, stepsPerMM_ = 0.0; 
        double current_pos_;
        AccelStepper motor_;
        const char* name_;
        int reset_pin_, fault_pin_, sleep_pin_;
        bool motor_fault = false;
        bool fault_block = false;
        bool moving = false;
        bool sleeping = false;


};

    
} // namespace hivepoker
#endif