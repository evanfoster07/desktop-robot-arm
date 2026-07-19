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