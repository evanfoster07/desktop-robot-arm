#include <Arduino.h>

#include "Config.h"
#include "RobotArmControl.h"
#include "SerialCommandHandler.h"

RobotArmControl robotArm{
    KinematicsConfig::ARM_GEOMETRY,
    JointMappingConfig::SHOULDER_CALIBRATION,
    JointMappingConfig::ELBOW_CALIBRATION,
    JointMappingConfig::WRIST_PITCH_CALIBRATION,
    JointMappingConfig::BASE_STEPS_PER_DEGREE
};

void setup()
{
    Serial.begin(115200);

    robotArm.begin();
    beginSerialCommands(robotArm);

    Serial.println("Robot arm controller ready");

}

void loop()
{
    readSerialCommands();
    robotArm.update();
}
