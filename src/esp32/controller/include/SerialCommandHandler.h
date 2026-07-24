#pragma once

class RobotArmControl;

/*
    Connects the serial command handler to the robot arm controller and
    initializes the input buffer.
*/
void beginSerialCommands(RobotArmControl& robotArm);

/*
    Reads available characters from the PC serial connection.
*/
void readSerialCommands();
