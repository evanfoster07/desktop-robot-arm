#include "SerialCommandHandler.h"

#include <Arduino.h>

#include "RobotArmControl.h"
#include "Poses.h"

namespace {

    String inputBuffer;
    RobotArmControl* robotArm = nullptr;

    bool requireValue(const String& command, bool hasValue) {
            if (hasValue) {
                return true;
            }

            Serial.print("Missing value for command: ");
            Serial.println(command);

            return false;
        }


    void executeCommand(const String& command, int value, bool hasValue) {

        // Commands that do not require a number can be handledbefore requireValue() checks.
        if (command == "home") {    
            robotArm->homeServos();
            Serial.println("Servos home");
            return;
        }

        if (command == "stop") {
            robotArm->stopBase();
            Serial.println("Base stopping");
            return;
        }

        if (command == "position") {
            Serial.print("Base position: ");
            Serial.println(robotArm->getBasePosition());
            return;
        }

        if (command == "fk") {
            /*
                Read the tracked physical actuator angles, reverse-map
                them into mathematical joint angles, and then run FK.
            */
            const JointAngles joints = robotArm->getCurrentJointAngles();
            const CartesianPose pose = robotArm->getCurrentPose();

            Serial.println();
            Serial.println("Current mathematical joint angles:");

            Serial.print("Base: ");
            Serial.println(joints.base);

            Serial.print("Shoulder: ");
            Serial.println(joints.shoulder);

            Serial.print("Elbow: ");
            Serial.println(joints.elbow);

            Serial.print("Wrist pitch: ");
            Serial.println(joints.wristPitch);

            Serial.println();
            Serial.println("Current Cartesian pose:");

            Serial.print("x: ");
            Serial.println(pose.x);

            Serial.print("y: ");
            Serial.println(pose.y);

            Serial.print("z: ");
            Serial.println(pose.z);

            Serial.print("pitch: ");
            Serial.println(pose.pitch);

            Serial.print("roll: ");
            Serial.println(pose.roll);

            Serial.println();
            return;
        }


        // Values formatted for Pi 
        if (command == "GET_STATE" || command == "get_state") {

            const JointAngles joints = robotArm->getCurrentJointAngles();
            const CartesianPose pose = robotArm->getCurrentPose();

            // Machine-readable state for Raspberry Pi
            Serial.print("STATE ");
            Serial.print(joints.base);
            Serial.print(" ");

            Serial.print(pose.x);
            Serial.print(" ");

            Serial.print(pose.y);
            Serial.print(" ");

            Serial.print(pose.z);
            Serial.print(" ");

            Serial.print(pose.pitch);
            Serial.print(" ");

            Serial.println(pose.roll);

            return;
        }

        
        // Commands below this point all require a numeric value

        if (!requireValue(command, hasValue)) {
            return;
        }


        
        // Normal angle commands

        if (command == "s") {
            robotArm->setServoAngle(ServoId::Shoulder, value);
        }
        else if (command == "e") {
            robotArm->setServoAngle(ServoId::Elbow, value);
        }
        else if (command == "p") {
            robotArm->setServoAngle(ServoId::WristPitch, value);
        }
        else if (command == "r") {
            robotArm->setServoAngle(ServoId::WristRoll, value);
        }
        else if (command == "g") {
            robotArm->setServoAngle(ServoId::Gripper, value);
        }


        
        // Direct pulse-width calibration commands

        else if (command == "scal") {
            robotArm->writeServoPulse(ServoId::Shoulder, value);
        }
        else if (command == "ecal") {
            robotArm->writeServoPulse(ServoId::Elbow, value);
        }
        else if (command == "pcal") {
            robotArm->writeServoPulse(ServoId::WristPitch, value);
        }
        else if (command == "rcal") {
            robotArm->writeServoPulse(ServoId::WristRoll, value);
        }
        else if (command == "gcal") {
            robotArm->writeServoPulse(ServoId::Gripper, value);
        }


        // Relative base movement in target steps
        else if (command == "base") {
            robotArm->moveBase(value);
        }

        // Preset position commands
        else if (command == "pose") {
            if (value < 0 || static_cast<size_t>(value) >= POSE_COUNT) {
                Serial.println("Invalid pose index");
                return;
            }

            robotArm->moveToPose(POSES[value]);
        }
        
        // Comm test
        else if (command == "hi") {
            Serial.print("HELLO ");
            Serial.println(value);
        }


        else {
            Serial.print("Unknown command: ");
            Serial.println(command);
        }
    }

    void executeCartesianPoseCommand(float x, float y, float z, float pitch, float roll) {
        CartesianPose target{
            x,
            y,
            z,
            pitch,
            roll
        };

        if (robotArm->moveToPose(target)) {
            Serial.println("POSE_OK");
        }
        else {
            Serial.println("POSE_FAIL");
        }
    }


    void processInputBuffer() {

        inputBuffer.trim();

        if (inputBuffer.length() == 0) {    // return if command is empty
            return;
        }

        const int spaceIndex = inputBuffer.indexOf(' ');    // Search for separator b/w command & value


        if (spaceIndex == -1) {     //If command has no value:

            const String command = inputBuffer;
            executeCommand(command, 0, false);
            return;
        }


        String command = inputBuffer.substring(0, spaceIndex);  // Extract text before value 
        command.trim();

        String valueString = inputBuffer.substring(spaceIndex + 1);     // Extract all text after space
        valueString.trim();


        if (valueString.length() == 0) {        // Reject an empty value string (e.g. "   ")
            executeCommand(command, 0, false);
            return;
        }

        // Cartesian pose command:
        // move_pose x y z pitch roll
        if (command == "move_pose") {

            float x;
            float y;
            float z;
            float pitch;
            float roll;

            const int valuesRead = sscanf(
                valueString.c_str(),
                "%f %f %f %f %f",
                &x,
                &y,
                &z,
                &pitch,
                &roll
            );

            // Five arguments means Cartesian pose command
            if (valuesRead == 5) {

                executeCartesianPoseCommand(
                    x,
                    y,
                    z,
                    pitch,
                    roll
                );

                return;
            }
        }

        const int value = valueString.toInt();  // Convert value to integer

        executeCommand(command, value, true);
    }
}

void beginSerialCommands(RobotArmControl& controller) {

    robotArm = &controller;

    inputBuffer.reserve(64);    // Reserve space in buffer to reduce repeated dynamic memory allocation
}


void readSerialCommands() {

    while (Serial.available() > 0) {

        const char incoming = Serial.read();

        if (incoming == '\n') {

            processInputBuffer();
            inputBuffer = "";
        }
        else {

            if (incoming != '\r') {     // Ignore carriage returns
                inputBuffer += incoming;
            }
        }
    }
}