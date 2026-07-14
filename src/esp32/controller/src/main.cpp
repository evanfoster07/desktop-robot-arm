#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

//Servo control pins
constexpr int SCL_PIN = 22;
constexpr int SDA_PIN = 21;

//PCA9685 servo channels
constexpr int shoulder = 0;
constexpr int elbow = 1;
constexpr int wrist_pitch = 2;
constexpr int wrist_roll = 3;
constexpr int gripper = 4;

//Stepper control pins
constexpr int EN_PIN = 25;
constexpr int UART_RX = 16;
constexpr int UART_TX = 17;
constexpr int STEP_PIN = 32;
constexpr int DIR_PIN = 33;

//Stepper address & sense resistor value
constexpr float R_SENSE = 0.11f;
constexpr int DRIVER_ADDRESS = 0;

//AccelStepper object
AccelStepper baseStepper{
    AccelStepper::DRIVER,
    STEP_PIN,
    DIR_PIN
};

//TMC2209 driver control object
TMC2209Stepper stepperDriver{
    &Serial2,
    R_SENSE,
    DRIVER_ADDRESS
};

String inputBuffer = "";

void executeCommand(const String& input) {
    if (input == "say hi") {
        Serial.println("HELLO");
    } else if (input == "left") {
        baseStepper.move(-800);
    } else if (input == "right") {
        baseStepper.move(800);
    } else if (input == "stop") {
        baseStepper.stop();
    }
}


void readSerialInput() {
    while (Serial.available() > 0) {
        char incoming = Serial.read();

        if (incoming == '\n') {
            inputBuffer.trim();
            if (inputBuffer.length() > 0) {
                executeCommand(inputBuffer);
            }

            inputBuffer = "";
        } else {
            inputBuffer += incoming;
        }
    }
}


void setup() {
    //PC Serial Monitor
    Serial.begin(115200);

    //TMC2209 UART
    Serial2.begin(
        115200,
        SERIAL_8N1,
        UART_RX,
        UART_TX
    );

    //TMC2209 enable pin
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);

    //Configure TMC2209
    stepperDriver.begin();

    stepperDriver.pdn_disable(true);
    stepperDriver.mstep_reg_select(true);
    stepperDriver.I_scale_analog(false);

    stepperDriver.toff(5);
    stepperDriver.rms_current(500);
    stepperDriver.microsteps(16); 
    stepperDriver.en_spreadCycle(false);
    stepperDriver.pwm_autoscale(true);

    //Configure stepper motion 
    baseStepper.setMaxSpeed(800);
    baseStepper.setAcceleration(300);
}

void loop() {
  readSerialInput();

  baseStepper.run();
}