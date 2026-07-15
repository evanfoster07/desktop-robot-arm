#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

//Servo control pins
constexpr int SCL_PIN = 22;
constexpr int SDA_PIN = 21;

//Stepper control pins
constexpr int EN_PIN = 25;
constexpr int UART_RX = 16;
constexpr int UART_TX = 17;
constexpr int STEP_PIN = 32;
constexpr int DIR_PIN = 33;

//Stepper address & sense resistor value
constexpr float R_SENSE = 0.11f;
constexpr int DRIVER_ADDRESS = 0;

//PCA9685 servo channels
constexpr int shoulder = 0;
constexpr int elbow = 1;
constexpr int wrist_pitch = 2;
constexpr int wrist_roll = 3;
constexpr int gripper = 4;

//Servo calibration constants 
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;

constexpr int ELBOW_MIN_US = 650;
constexpr int ELBOW_MAX_US = 2630;

constexpr int SERVO_FREQUENCY = 50;     // 50 Hz

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

//Servo driver control object 
Adafruit_PWMServoDriver servoDriver{
    0x40,
    Wire
};

//Servo struct & array for variable tracking
struct ServoJoint {
    const char* name;
    uint8_t channel;

    int homeAngle;
    int currAngle;

    int minAngle;
    int maxAngle;

    int minPulseUS;
    int maxPulseUS;
};

ServoJoint servos[] = {
    {"shoulder", 0, 70, 70, 0, 180, SERVO_MIN_US, SERVO_MAX_US},
    {"elbow", 1, 110, 110, 0, 180, SERVO_MIN_US, SERVO_MAX_US},
    {"wrist_pitch", 2, 60, 60, 0, 180, SERVO_MIN_US, SERVO_MAX_US},
    {"wrist_roll", 3, 90, 90, 0, 180, SERVO_MIN_US, SERVO_MAX_US},
    {"gripper", 4, 80, 80, 0, 80, SERVO_MIN_US, SERVO_MAX_US}
};


void setServoAngle(ServoJoint& servo, int requestedAngle) {
    int safeAngle = constrain(
        requestedAngle,
        servo.minAngle,
        servo.maxAngle
    );

    int pulseWidth = map(
        safeAngle,
        servo.minAngle,
        servo.maxAngle,
        servo.minPulseUS,
        servo.maxPulseUS
    );

    servoDriver.writeMicroseconds(servo.channel, pulseWidth);
    servo.currAngle = safeAngle;
}

String inputBuffer = "";

void executeCommand(const String& command, int value) {
    if (command == "hi") {
        Serial.print("HELLO");
        Serial.println(value);
    } 
    else if (command == "s") {
        setServoAngle(servos[0], value);
    }
    else if (command == "e") {
        setServoAngle(servos[1], value);
    }
    else if (command == "p") {
        setServoAngle(servos[2], value);
    }
    else if (command == "r") {
        setServoAngle(servos[3], value);
    }
    else if (command == "g") {
        setServoAngle(servos[4], value);
    }

    // calibration
    else if (command == "scal") {
        servoDriver.writeMicroseconds(servos[0].channel, value);
    }
    else if (command == "ecal") {
        servoDriver.writeMicroseconds(servos[1].channel, value);
    }
    else if (command == "pcal") {
        servoDriver.writeMicroseconds(servos[2].channel, value);
    }
    else if (command == "rcal") {
        servoDriver.writeMicroseconds(servos[3].channel, value);
    }
    else if (command == "gcal") {
        servoDriver.writeMicroseconds(servos[4].channel, value);
    }

    else if (command == "base") {
        baseStepper.move(value);
    }
}

void readSerialInput() {
    while (Serial.available() > 0) {
        char incoming = Serial.read();

        if (incoming == '\n') {
            inputBuffer.trim();
            if (inputBuffer.length() > 0) {
                int spaceIndex = inputBuffer.indexOf(' ');
                String command = inputBuffer.substring(0, spaceIndex);

                String valueString = inputBuffer.substring(spaceIndex + 1);
                int value = valueString.toInt();

                executeCommand(command, value);
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

    //Initialize PCA9685 I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    //Configure PCA9685
    servoDriver.begin();
    servoDriver.setPWMFreq(SERVO_FREQUENCY);
}

void loop() {
  readSerialInput();

  baseStepper.run();
}