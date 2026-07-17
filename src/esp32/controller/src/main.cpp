#include <Arduino.h>

#include "ServoController.h"
#include "StepperController.h"
#include "SerialCommandHandler.h"

void setup() {
    
    Serial.begin(115200);

    beginServos();
    beginStepper();
    beginSerialCommands();

    Serial.println("Robot arm controller ready");
}

void loop() {

    readSerialCommands();
    updateStepper();
}