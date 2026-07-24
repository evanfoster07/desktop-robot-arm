#include <Arduino.h>

#include "Config.h"
#include "RobotArmControl.h"
#include "SerialCommandHandler.h"

RobotArmControl robotArm{
    KinematicsConfig::ARM_GEOMETRY
};

void setup()
{
    Serial.begin(115200);

    robotArm.begin();
    beginSerialCommands(robotArm);

    Serial.println("Robot arm controller ready");

    // Temporary IK/FK verification test
    CartesianPose target;

    target.x = 200.0f;
    target.y = 50.0f;
    target.z = 300.0f;
    target.pitch = 50.0f;
    target.roll = 0.0f;

    JointAngles solution;

    const bool reachable =
        robotArm.solveInverseKinematics(target, solution);

    if (!reachable)
    {
        Serial.println("Target unreachable.");
        return;
    }

    Serial.println("Solution:");

    Serial.print("base: ");
    Serial.println(solution.base);

    Serial.print("shoulder: ");
    Serial.println(solution.shoulder);

    Serial.print("elbow: ");
    Serial.println(solution.elbow);

    Serial.print("wristPitch: ");
    Serial.println(solution.wristPitch);

    Serial.print("wristRoll: ");
    Serial.println(solution.wristRoll);

    const CartesianPose verifiedPose =
        robotArm.solveForwardKinematics(solution);

    Serial.println("FK verification:");

    Serial.print("X: ");
    Serial.println(verifiedPose.x);

    Serial.print("Y: ");
    Serial.println(verifiedPose.y);

    Serial.print("Z: ");
    Serial.println(verifiedPose.z);

    Serial.print("Pitch: ");
    Serial.println(verifiedPose.pitch);

    Serial.print("Roll: ");
    Serial.println(verifiedPose.roll);
}

void loop()
{
    readSerialCommands();
    robotArm.update();
}
