#include "SerialCommandHandler.h"

#include <Arduino.h>

#include "ServoController.h"
#include "StepperController.h"

namespace {

    String inputBuffer;

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
            homeServos();
            Serial.println("Servos home");
            return;
        }

        if (command == "stop") {
            stopBase();
            Serial.println("Base stopping");
            return;
        }

        if (command == "position") {
            Serial.print("Base position: ");
            Serial.println(getBasePosition());
            return;
        }


        
        // Commands below this point all require a numeric value

        if (!requireValue(command, hasValue)) {
            return;
        }


        
        // Normal angle commands

        if (command == "s") {
            setServoAngle(ServoId::Shoulder, value);
        }
        else if (command == "e") {
            setServoAngle(ServoId::Elbow, value);
        }
        else if (command == "p") {
            setServoAngle(ServoId::WristPitch, value);
        }
        else if (command == "r") {
            setServoAngle(ServoId::WristRoll, value);
        }
        else if (command == "g") {
            setServoAngle(ServoId::Gripper, value);
        }


        
        // Direct pulse-width calibration commands

        else if (command == "scal") {
            writeServoPulse(ServoId::Shoulder, value);
        }
        else if (command == "ecal") {
            writeServoPulse(ServoId::Elbow, value);
        }
        else if (command == "pcal") {
            writeServoPulse(ServoId::WristPitch, value);
        }
        else if (command == "rcal") {
            writeServoPulse(ServoId::WristRoll, value);
        }
        else if (command == "gcal") {
            writeServoPulse(ServoId::Gripper, value);
        }


        // Relative base movement in target steps
        else if (command == "base") {
            moveBase(value);
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


        const int value = valueString.toInt();  // Convert value to integer

        executeCommand(command, value, true);
    }
}

void beginSerialCommands() {

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