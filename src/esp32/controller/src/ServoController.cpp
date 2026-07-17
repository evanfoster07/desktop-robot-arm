#include "ServoController.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "Config.h"

namespace {

    // Servo calibration constants 
    constexpr int SHOULDER_MIN_US = 800;
    constexpr int SHOULDER_MAX_US = 2420;

    constexpr int ELBOW_MIN_US = 650;
    constexpr int ELBOW_MAX_US = 2630;

    constexpr int WPITCH_MIN_US = 550;
    constexpr int WPITCH_MAX_US = 2620;

    constexpr int WROLL_MIN_US = 500;
    constexpr int WROLL_MAX_US = 2620;

    constexpr int GRIPPER_MIN_US = 500;
    constexpr int GRIPPER_MAX_US = 1550;


    // PCA9685 control object
    Adafruit_PWMServoDriver servoDriver{
        ServoConfig::PCA9685_ADDRESS,
        Wire
    };


    // Converts ServoId enum into ordinary array index
    constexpr size_t servoIndex(ServoId servoId) {
        return static_cast<size_t>(servoId);
    }


    // Servo order matches order in ServoId
    ServoJoint servos[] = {
        {"shoulder", 0, 145, 145, 20, 160, SHOULDER_MIN_US, SHOULDER_MAX_US},
        {"elbow", 1, 25, 25, 0, 180, ELBOW_MIN_US, ELBOW_MAX_US},
        {"wrist_pitch", 2, 20, 20, 0, 180, WPITCH_MIN_US, WPITCH_MAX_US},
        {"wrist_roll", 3, 90, 90, 0, 180, WROLL_MIN_US, WROLL_MAX_US},
        {"gripper", 4, 80, 80, 0, 80, GRIPPER_MIN_US, GRIPPER_MAX_US}
    };


    // Number of physical servos
    constexpr size_t SERVO_COUNT =
        static_cast<size_t>(ServoId::Count);
}


void beginServos() {
    Wire.begin(Pins::SDA_PIN, Pins::SCL_PIN);

    servoDriver.begin();
    servoDriver.setPWMFreq(ServoConfig::PWM_FREQUENCY);
}

void setServoAngle(ServoId servoId, int requestedAngle) {

    ServoJoint& servo = servos[servoIndex(servoId)];    // Reference to ServoJoint object

    const int safeAngle = constrain(
        requestedAngle,
        servo.minAngle,
        servo.maxAngle
    );
    

    int pulseWidthUS = map(
        safeAngle,
        servo.minAngle,
        servo.maxAngle,
        servo.minPulseUS,
        servo.maxPulseUS
    );

    servoDriver.writeMicroseconds(servo.channel, pulseWidthUS);
    servo.currAngle = safeAngle;
}


void writeServoPulse(ServoId servoId, int pulseWidthUS) {

    const ServoJoint& servo = servos[servoIndex(servoId)];  // Read-only

    servoDriver.writeMicroseconds(servo.channel, pulseWidthUS);
}


void homeServos() {

    for (size_t index = 0; index < SERVO_COUNT; index++) {

        const ServoId servoId = static_cast<ServoId>(index);    // Convert index back into ServoId

        setServoAngle(servoId, servos[index].homeAngle);
    }
}


const ServoJoint& getServoJoint(ServoId servoId) {

    return servos[servoIndex(servoId)];
}