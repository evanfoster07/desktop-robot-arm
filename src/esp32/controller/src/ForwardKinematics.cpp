#include "ForwardKinematics.h"

#include <cmath>


// Define pi for calculations
namespace 
{
    constexpr float PI = 3.14159265358979323846264;
}


ForwardKinematics::ForwardKinematics(const ArmGeometry& geometry) : geometry(geometry) {}


CartesianPose ForwardKinematics::solve(const JointAngles& joints) const
{
    // Convert all independent angles from degrees to radians
    const float baseRads = degToRads(joints.base);
    const float shoulderRads = degToRads(joints.shoulder);

    // Forearm angle = shoulder + elbow angles 
    const float forearmDegs = joints.shoulder + joints.elbow;
    const float forearmRads = degToRads(forearmDegs); 

    // Wrist/tool angle = shoulder + elbow + wristPitch angles 
    const float toolPitchDegs = joints.shoulder + joints.elbow + joints.wristPitch;
    const float toolPitchRads = degToRads(toolPitchDegs);


    // Calculate horizontal contribution of each arm segment
    const float upperArmRadial = geometry.upperArmLength * std::cos(shoulderRads);
    const float forearmRadial = geometry.forearmLength * std::cos(forearmRads);
    const float wristRadial = geometry.wristLength * std::cos(toolPitchRads);

    // Total distance outwards from base vertical axis
    const float radialDist = upperArmRadial + forearmRadial + wristRadial;

    // Calculate vertical contribution of each arm segment
    const float upperArmVertical = geometry.upperArmLength * std::sin(shoulderRads);
    const float forearmVertical = geometry.forearmLength * std::sin(forearmRads);
    const float wristVertical = geometry.wristLength * std::sin(toolPitchRads);


    CartesianPose pose;

    // Calculate x and y coordinates of gripper
    pose.x = radialDist * std::cos(baseRads);
    pose.y = radialDist * std::sin(baseRads);

    // z coordinate is sum of vertical components + base height
    pose.z = upperArmVertical + forearmVertical + wristVertical + geometry.baseHeight;

    pose.pitch = toolPitchDegs;
    pose.roll = joints.wristRoll;   // Wrist roll is unaffected 

    return pose;
}


float ForwardKinematics::radsToDeg(float radians) 
{
    return radians * 180.f / PI;
}

float ForwardKinematics::degToRads(float degrees)
{
    return degrees * PI / 180.f;
}