#pragma once


/*
    Describes the physical dimensions of the robot arm in millimeters
*/
struct ArmGeometry 
{
    float baseHeight;
    float upperArmLength;
    float forearmLength;
    float wristLength;
};


/*
    Describes the mathematical angle of each joint 
    
    These are not necessarily the same as raw servo angles
*/
struct JointAngles 
{
    float base;
    float shoulder; 
    float elbow;
    float wristPitch;
    float wristRoll;
};


/*
    Describes where the gripper tip is located in Cartesian space

    Position:
        x, y, z
    
    Orientation:
        pitch = tilt up/down
        roll = rotation around gripper axis
*/
struct CartesianPose 
{
    float x;
    float y;
    float z;

    float pitch;
    float roll;
};


/*
    Describes how one mathematical robot joint maps to its actuator

    mathReferenceAngle:
        A known mathematical joint angle

    actuatorReferenceAngle:
        The raw actuator angle that produces that mathematical angle

    direction:
        +1 if increasing the mathematical angle increases the actuator angle
        -1 if increasing the mathematical angle decreases the actuator angle
*/
struct JointCalibration
{
    float mathReferenceAngle;
    float actuatorReferenceAngle;
    float direction;
};


/*
    Stores the raw actuator commands produced by the joint mapper

    These are not mathematical robot-joint angles

    The servo values are raw angles that can be passed to ServoController
    The base value is the target stepper position in steps
*/
struct ActuatorTargets
{
    long baseSteps;

    float shoulderServoAngle;
    float elbowServoAngle;
    float wristPitchServoAngle;
};