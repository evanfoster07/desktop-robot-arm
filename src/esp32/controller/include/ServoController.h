#pragma once

#include <Arduino.h>


/*
    ServoId gives each servo a readable identifier.
*/
enum class ServoId : uint8_t {
    Shoulder,
    Elbow,
    WristPitch,
    WristRoll,
    Gripper,
    Count   // # of servo entries
};


/*
    Servo struct & array for variable tracking
*/
struct ServoJoint {
    const char* name;
    uint8_t channel;
    int homeAngle;

    float currAngle;
    float targetAngle;

    int minAngle;
    int maxAngle;

    int minPulseUS;
    int maxPulseUS;

    bool interpolationEnabled;
    float speedDegPerSec;
};


/*
    Initializes I2C & PCA9685 servo driver
*/
void beginServos();


/*
    Moves a servo to a joint angle

    The requested angle is constrained to the servo's allowed range of motion,
    then mapped to its calibrated pulse-width range
*/
void setServoAngle(ServoId servoId, int requestedAngle);


/*
    Writes a pulse width directly to a servo. Primarily used for calibration
*/
void writeServoPulse(ServoId servoId, int pulseWidthUS);


/*
    Advances servos with interpolation toward their target angles

    Must be called repeatedly from loop()
*/
void updateServos();


/*
    Moves every servo to its individually configured home angle.
*/
void homeServos();


/*
    Returns a read-only reference to a joint's stored data.

    const prevents code outside this module from accidentally changing
    the calibration or current-angle data directly.
*/
const ServoJoint& getServoJoint(ServoId servoId);