#include "StepperController.h"

#include <AccelStepper.h>
#include <TMCStepper.h>

#include "Config.h"

namespace {
    // AccelStepper object declaration
    AccelStepper baseStepper{   
        AccelStepper::DRIVER,
        Pins::STEP_PIN,
        Pins::DIR_PIN
    };

    // TMCStepper object declaration
    TMC2209Stepper stepperDriver{
        &Serial2,   // ESP32's second UART object for communication
        StepperConfig::R_SENSE,
        StepperConfig::DRIVER_ADDRESS
    };
}


void beginStepper() {

    // Configure UART for ESP32 -> TMC2209 communication
    Serial2.begin(StepperConfig::UART_BAUD_RATE, SERIAL_8N1, Pins::UART_RX, Pins::UART_TX);

    pinMode(Pins::STEPPER_EN, OUTPUT);  
    digitalWrite(Pins::STEPPER_EN, LOW);    // TMC2209 EN active low

    stepperDriver.begin();


    stepperDriver.pdn_disable(true);    // Disable the normal PDN function to ensure pin is used for UART
    stepperDriver.mstep_reg_select(true);   // Select microstep settings through UART instead of physical pins
    stepperDriver.I_scale_analog(false);    // false selects UART-controlled current scaling
    stepperDriver.toff(5);  // Enable driver motor outputs

    stepperDriver.rms_current(StepperConfig::RMS_CURRENT_MA);
    stepperDriver.microsteps(StepperConfig::MICROSTEPS);

    stepperDriver.en_spreadCycle(false);    // false enables StealthChop operation rather than SpreadCycle
    stepperDriver.pwm_autoscale(true);  //  Allows the driver to automatically tune StealthChop PWM

    baseStepper.setMaxSpeed(StepperConfig::MAX_SPEED);
    baseStepper.setAcceleration(StepperConfig::ACCELERATION);


    Serial.print("TMC connection: ");
    Serial.println(stepperDriver.test_connection());

    Serial.print("Configured microsteps: ");
    Serial.println(stepperDriver.microsteps());

    Serial.print("MSTEP register select: ");
    Serial.println(stepperDriver.mstep_reg_select());
}


void moveBase(long relativeSteps) {

    baseStepper.move(relativeSteps);
}


void stopBase() {

    baseStepper.stop();     // Changes step target so motor decelerates to rest
}


void updateStepper() {

    baseStepper.run();      // Should be called as frequently as possible
}


long getBasePosition() {
    
    return baseStepper.currentPosition();
}
