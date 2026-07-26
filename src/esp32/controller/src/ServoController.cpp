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
        {"shoulder", 0, 145, 145.f, 145.f, 20, 160, SHOULDER_MIN_US, SHOULDER_MAX_US, true, 40.f},
        {"elbow", 1, 25, 25.f, 25.f, 0, 180, ELBOW_MIN_US, ELBOW_MAX_US, true, 50.f},
        {"wrist_pitch", 2, 20, 20.f, 20.f, 0, 180, WPITCH_MIN_US, WPITCH_MAX_US, true, 60.f},
        {"wrist_roll", 3, 90, 90.f, 90.f, 0, 180, WROLL_MIN_US, WROLL_MAX_US, true, 60.f},
        {"gripper", 4, 80, 80.f, 80.f, 0, 80, GRIPPER_MIN_US, GRIPPER_MAX_US, false, 50.f}
    };


    // Number of physical servos
    constexpr size_t SERVO_COUNT =
        static_cast<size_t>(ServoId::Count);


    // Private helper function for angle interpolation
    void writeServoAngle(ServoJoint& servo, float angle) {  

        const int roundedAngle = round(angle);
        
        const int pulseWidthUS = map(
            roundedAngle,
            servo.minAngle,
            servo.maxAngle,
            servo.minPulseUS,
            servo.maxPulseUS
        );

        servoDriver.writeMicroseconds(servo.channel, pulseWidthUS);
    }
}


void beginServos() {
    Wire.begin(Pins::SDA_PIN, Pins::SCL_PIN);

    servoDriver.begin();
    servoDriver.setPWMFreq(ServoConfig::PWM_FREQUENCY);
}


void setServoAngle(ServoId servoId, int requestedAngle) {
    ServoJoint& servo = servos[servoIndex(servoId)];

    const int safeAngle = constrain(requestedAngle, servo.minAngle, servo.maxAngle);

    if (servo.interpolationEnabled) {
        servo.targetAngle = safeAngle;
    } else {    
        servo.currAngle = safeAngle;
        servo.targetAngle = safeAngle;

        writeServoAngle(servo, servo.currAngle);
    }
}


void writeServoPulse(ServoId servoId, int pulseWidthUS) {

    const ServoJoint& servo = servos[servoIndex(servoId)];  // Read-only

    servoDriver.writeMicroseconds(servo.channel, pulseWidthUS);
}


void updateServos() {
    static unsigned long prevUpdateMS = millis(); 

    const unsigned long currTimeMS = millis();
    const unsigned long elapsedTimeMS = currTimeMS - prevUpdateMS;  

    prevUpdateMS = currTimeMS;

    const float elapsedSeconds = elapsedTimeMS / 1000.0f;

    for (size_t index = 0; index < SERVO_COUNT; index++) {

        ServoJoint& servo = servos[index];

        if (!servo.interpolationEnabled) {
            continue;
        }

        const float angleDifference = servo.targetAngle - servo.currAngle;
        const float maxMovement = servo.speedDegPerSec * elapsedSeconds;

        if (abs(angleDifference) <= maxMovement) {      // Snap to final position if it can be reached 
            servo.currAngle = servo.targetAngle;
        }
        else if (angleDifference > 0.0f) {    // If target is above current position
            servo.currAngle += maxMovement;
        } else {                              // If target is behind current positon
            servo.currAngle -= maxMovement;
        }

        writeServoAngle(servo, servo.currAngle);
    }
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

bool isServoAngleValid(ServoId servoId, float requestedAngle)
{
    const ServoJoint& servo = servos[servoIndex(servoId)];
    
    return requestedAngle >= servo.minAngle && requestedAngle <= servo.maxAngle;
}