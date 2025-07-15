#include <hivepoker_controller.hpp>


hivepoker::HivepokerController::HivepokerController() {
    // your initialization code here
}


void hivepoker::HivepokerController::init(int arms_number, bool inverted_endstop) {

    if (arms_number > 2 || arms_number <= 0) {
        hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "[HivepokerController] - Number of arms set to %d; Not supported", arms_number);
        return;
    }

    instance = this;

    arms_number_ = arms_number;
    inverted_endstop_ = inverted_endstop;

    x1_driver.init(
        X1_STEP_PIN, 
        X1_DIR_PIN, 
        X1_RST_PIN, 
        X1_FAULT_PIN,
        X1_SLEEP_PIN,
        X_STEPS_PER_REVOLUTION,
        X_MM_TRAVELED_PER_REVOLUTION,
        "X1"
    );


    y1_driver.init(
        Y1_STEP_PIN, 
        Y1_DIR_PIN, 
        Y1_RST_PIN, 
        Y1_FAULT_PIN,
        Y1_SLEEP_PIN,
        Y_STEPS_PER_REVOLUTION,
        Y_MM_TRAVELED_PER_REVOLUTION,
        "Y1"
    );

    if (arms_number_ == 2) {

        x2_driver.init(
            X2_STEP_PIN, 
            X2_DIR_PIN, 
            X2_RST_PIN, 
            X2_FAULT_PIN,
            X2_SLEEP_PIN,
            X_STEPS_PER_REVOLUTION,
            X_MM_TRAVELED_PER_REVOLUTION,
            "X2"
        );


        y2_driver.init(
            Y2_STEP_PIN, 
            Y2_DIR_PIN, 
            Y2_RST_PIN, 
            Y2_FAULT_PIN,
            Y2_SLEEP_PIN,
            Y_STEPS_PER_REVOLUTION,
            Y_MM_TRAVELED_PER_REVOLUTION,
            "Y2"
        );

    }


    initialized = true;
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "[HivepokerController] - Controller initialized with %d arms", arms_number_);    

}




void hivepoker::HivepokerController::attachLeftEndstop() {

    if (!initialized) {
        hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "[HivepokerController] - Controller has not been initialized yet!");  
        return;
    }

    pinMode(LEFT_ENDSTOP_PIN, INPUT);
    xTaskCreatePinnedToCore(taskWrapperLeft, "EndstopTaskLeft", 2048, this, 1, &taskHandleL, 1);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENDSTOP_PIN), isrWrapperLeft, CHANGE);

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "[HivepokerController] - Attached left endstop interrupt");

}

void IRAM_ATTR hivepoker::HivepokerController::isrWrapperLeft() {
    if (instance) instance->handleInterruptLeft();
}

void hivepoker::HivepokerController::taskWrapperLeft(void* param) {
    static_cast<HivepokerController*>(param)->endstopTaskLeft();
}

void IRAM_ATTR hivepoker::HivepokerController::handleInterruptLeft() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandleL, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void hivepoker::HivepokerController::endstopTaskLeft() {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        endstop_l = !endstop_l;
    }
}







void hivepoker::HivepokerController::attachRightEndstop() {

    if (!initialized) {
        hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "[HivepokerController] - Controller has not been initialized yet!");  
        return;
    }

    pinMode(RIGHT_ENDSTOP_PIN, INPUT);
    xTaskCreatePinnedToCore(taskWrapperRight, "EndstopTaskRight", 2048, this, 1, &taskHandleR, 1);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENDSTOP_PIN), isrWrapperRight, CHANGE);

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "[HivepokerController] - Attached right endstop interrupt");

}

void IRAM_ATTR hivepoker::HivepokerController::isrWrapperRight() {
    if (instance) instance->handleInterruptRight();
}

void hivepoker::HivepokerController::taskWrapperRight(void* param) {
    static_cast<HivepokerController*>(param)->endstopTaskRight();
}

void IRAM_ATTR hivepoker::HivepokerController::handleInterruptRight() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandleR, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void hivepoker::HivepokerController::endstopTaskRight() {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        endstop_r = !endstop_r;
    }
}









void hivepoker::HivepokerController::attachMiddleEndstop() {

    if (!initialized) {
        hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "[HivepokerController] - Controller has not been initialized yet!");  
        return;
    }

    pinMode(MIDDLE_ENDSTOP_PIN, INPUT);
    xTaskCreatePinnedToCore(taskWrapperMiddle, "EndstopTaskMiddle", 2048, this, 1, &taskHandleM, 1);
    attachInterrupt(digitalPinToInterrupt(MIDDLE_ENDSTOP_PIN), isrWrapperMiddle, CHANGE);

    hivepoker::Logger::log(hivepoker::Logger::LogLevel::INFO, "[HivepokerController] - Attached middle endstop interrupt");

}

void IRAM_ATTR hivepoker::HivepokerController::isrWrapperMiddle() {
    if (instance) instance->handleInterruptMiddle();
}

void hivepoker::HivepokerController::taskWrapperMiddle(void* param) {
    static_cast<HivepokerController*>(param)->endstopTaskMiddle();
}

void IRAM_ATTR hivepoker::HivepokerController::handleInterruptMiddle() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandleM, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void hivepoker::HivepokerController::endstopTaskMiddle() {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        endstop_m = !endstop_m;
    }
}








void hivepoker::HivepokerController::run() {

    if (!initialized) {
        hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "[HivepokerController] - Controller has not been initialized yet!");  
        return;
    }

    x1_driver.run();
    y1_driver.run();

    if (arms_number_ == 2) {
        x2_driver.run();
        y2_driver.run();
    }

}





void hivepoker::HivepokerController::MoveArm1(float x, float y) {

    if (!initialized) {
        hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "[HivepokerController] - Controller has not been initialized yet!");  
        return;
    }
    hivepoker::Logger::log(hivepoker::Logger::LogLevel::DEBUG, "[HivepokerController] - Moving Arm1 to position X: %f, Y: %f", x, y); 
    x1_driver.moveTo(x);
    y1_driver.moveTo(y);

}


bool hivepoker::HivepokerController::isArm1GoalReached() {

    if (!initialized) {
        hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "[HivepokerController] - Controller has not been initialized yet!");  
        return false;
    }

    if (x1_driver.isMoving() || y1_driver.isMoving()) {
        return false;
    }

    return true;
}


std::tuple<float, float> hivepoker::HivepokerController::getArm1Position() {

    if (!initialized) {
        hivepoker::Logger::log(hivepoker::Logger::LogLevel::ERROR, "[HivepokerController] - Controller has not been initialized yet!");  
        return std::make_tuple(NAN, NAN);
    }

    float x = x1_driver.getCurrentPosition();
    float y = y1_driver.getCurrentPosition();

    return std::make_tuple(x,y);

}