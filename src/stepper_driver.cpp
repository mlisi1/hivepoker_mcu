#include <stepper_driver.hpp>

hivepoker::StepperDriver::StepperDriver() {
    
}

void hivepoker::StepperDriver::init(int step_pin, int dir_pin, int reset_pin, int fault_pin, int sleep_pin, float stepsPerRev, float travelPerRev_mm, const char* name) {

    motor_ = AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin);
    stepsPerRev_ = stepsPerRev;
    travelPerRev_mm_ = travelPerRev_mm;
    stepsPerMM_ = stepsPerRev / travelPerRev_mm;
    name_ = name;
    reset_pin_ = reset_pin;
    fault_pin_ = fault_pin;
    sleep_pin_ = sleep_pin;

    motor_.setMaxSpeed(500); 

    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, HIGH);
    pinMode(fault_pin, INPUT_PULLUP);
    pinMode(sleep_pin, OUTPUT);
    digitalWrite(sleep_pin, HIGH);

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "Stepper %s initialized", name);

}



void hivepoker::StepperDriver::moveTo(double pos) {

    long steps = pos * stepsPerMM_;
    motor_.moveTo(steps);

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "[%s] Moving to position %d", name_, pos);

}

float hivepoker::StepperDriver::getCurrentPosition() {

    return current_pos_;

}

void hivepoker::StepperDriver::run() {

    checkMotorState();

    if (motor_fault && fault_block) {
        return;
    }

    if (!motor_fault && fault_block) {
        recoveryProcedure();
    }

    if (motor_fault) {
        motor_.stop();
        fault_block = true;
        hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "[%s] Fault detected", name_);
    }

    moving = motor_.run();
    current_pos_ = motor_.currentPosition() / stepsPerMM_;

}


void hivepoker::StepperDriver::stopMotor() {

    motor_.stop();
    moving = false;

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "[%s] Stopping", name_);

}


void hivepoker::StepperDriver::checkMotorState() {

    motor_fault = ((fault_pin_) == LOW);

}


void hivepoker::StepperDriver::recoveryProcedure() {

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::WARNING, "[%s] Starting recovery procedure", name_);

    digitalWrite(reset_pin_, LOW);

    delay(100);

    digitalWrite(reset_pin_, HIGH);

    fault_block = false;

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "[%s] Recovery procedure completed", name_);

}



bool hivepoker::StepperDriver::isMoving() {
    return moving;
}


void hivepoker::StepperDriver::sleep() {

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "[%s] Going to sleep", name_);

    digitalWrite(sleep_pin_, LOW);
    sleeping = true;

}



void hivepoker::StepperDriver::wakeUp() {

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "[%s] WakingUp", name_);

    digitalWrite(sleep_pin_, HIGH);
    sleeping = false;

}


bool hivepoker::StepperDriver::isSleeping() {

    return sleeping;

}