#include "InverseKinematics.h"

#include <cmath>
#include <Arduino.h>


// Define pi for calculations
namespace 
{
    constexpr float PI_VALUE = 3.14159265358979323846264;

    // small tolerance used for checking floating-point vals
    constexpr float REACH_TOLERANCE = 0.0001f;
}

InverseKinematics::InverseKinematics(const ArmGeometry& geometry) : geometry(geometry) {}

bool InverseKinematics::solve(const CartesianPose& target, JointAngles& solution, bool preferElbowPositive) const
{
    // Convert desired gripper pitch to radians
    // Represents absolute angle of gripper relative to horizontal plane
    const float targetPitchRads = degToRads(target.pitch);

    // Calculate base angle 
    const float baseRads = std::atan2(target.y, target.x);

    // Calculate radial distance
    const float radialDistance = std::sqrt(target.x * target.x + target.y * target.y);

    // Subtract wrist contribution from the target position to solve for wrist joint position 
    const float wristVert = target.z - geometry.baseHeight - geometry.wristLength  * std::sin(targetPitchRads);
    const float wristRadial = radialDistance - geometry.wristLength * std::cos(targetPitchRads);
    const float shoulderToWristSquared = wristRadial * wristRadial + wristVert * wristVert;

    /*
        Calculate elbow angle with law of cosines (+ clockwise, 0° = straight forearm relative to upper arm)

        Rearranged eq: 
            cos(elbow) = (shoulderToWrist^2 - upperArm^2 - forearm^2) / (2 * upperArm * forearm)
    */
    const float numerator = shoulderToWristSquared - geometry.upperArmLength * geometry.upperArmLength -
        geometry.forearmLength * geometry.forearmLength;
    const float denominator = 2.0f * geometry.upperArmLength * geometry.forearmLength;

    // Protect against invalid geometry
    if (denominator == 0.0f) 
    {
        return false;
    }

    float cosElbow = numerator / denominator;

    // For a real angle: -1 <= cosElbow <= 1
    if (cosElbow < -1.0f - REACH_TOLERANCE || cosElbow > 1.0f + REACH_TOLERANCE)
    {
        return false;
    }

    // Clamp floating-point errors before calling acos()
    cosElbow = constrain(cosElbow, -1.0f, 1.0f);
    const float elbowMagnitudeRads = std::acos(cosElbow);


    /*
        Calculate shoulder angle (0° when horizontal along +x axis)
        
        Made of two angles:
            - targetDirection: angle from shoulder directly towards wrist
            - elbowOffset: offset caused by upper arm/forearm triangle

        shoulder = targetDirection - elbowOffset
    */
    const float targetDirection = std::atan2(wristVert, wristRadial);
    const float elbowOffset = std::atan2(
        geometry.forearmLength * std::sin(elbowMagnitudeRads),
        geometry.upperArmLength + geometry.forearmLength * std::cos(elbowMagnitudeRads)
    );

    const float elbowRads = preferElbowPositive
        ? elbowMagnitudeRads
        : -elbowMagnitudeRads;

    const float shoulderRads = preferElbowPositive
        ? targetDirection - elbowOffset
        : targetDirection + elbowOffset;

    /*
        Calculate wrist-pitch joint angle (+ clockwise, 0° = straight wrist relative to forearm)

        wristPitch = gripper target - shoulder - elbow
    */
   const float wristPitchRads = targetPitchRads - shoulderRads - elbowRads;

   // Store solution 
   solution.base = radsToDeg(baseRads);
   solution.shoulder = radsToDeg(shoulderRads);
   solution.elbow = radsToDeg(elbowRads);
   solution.wristPitch = radsToDeg(wristPitchRads);
   solution.wristRoll = target.roll;

   return true;     // Must be a valid solution at this point
}


float InverseKinematics::radsToDeg(float radians) 
{
    return radians * 180.f / PI_VALUE;
}

float InverseKinematics::degToRads(float degrees)
{
    return degrees * PI_VALUE / 180.f;
}