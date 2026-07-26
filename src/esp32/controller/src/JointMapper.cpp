#include "JointMapper.h"
#include "ServoController.h"

#include <cmath>


JointMapper::JointMapper(
    const JointCalibration& shoulderCalibration, 
    const JointCalibration& elbowCalibration,
    const JointCalibration& wristPitchCalibration,
    float baseStepsPerDegree
    ) 
    :   shoulderCalibration(shoulderCalibration), 
        elbowCalibration(elbowCalibration), 
        wristPitchCalibration(wristPitchCalibration), 
        baseStepsPerDegree(baseStepsPerDegree) {}


bool JointMapper::map(const JointAngles& jointAngles, ActuatorTargets& targets) const
{
    // Convert mathematical base angle into an absolute stepper position
    targets.baseSteps = std::lround(jointAngles.base * baseStepsPerDegree);

    // Convert each mathematical servo joint angle into the corresponding servo command
    targets.shoulderServoAngle = mapServoAngle(jointAngles.shoulder, shoulderCalibration);
    targets.elbowServoAngle = mapServoAngle(jointAngles.elbow, elbowCalibration);
    targets.wristPitchServoAngle = mapServoAngle(jointAngles.wristPitch, wristPitchCalibration);

     /*
        Check the mapped angles against the safe ranges inside ServoController
        
        If one angle is invalid, the complete Cartesian pose is rejected
    */
    const bool shoulderValid = isServoAngleValid(ServoId::Shoulder, targets.shoulderServoAngle);
    const bool elbowValid = isServoAngleValid(ServoId::Elbow, targets.elbowServoAngle);
    const bool wristPitchValid = isServoAngleValid(ServoId::WristPitch,targets.wristPitchServoAngle);

    return shoulderValid && elbowValid && wristPitchValid;
}

float JointMapper::mapServoAngle(float mathAngle, const JointCalibration& calibration) const 
{
    // Find how far the mathematical angle is from the reference pose
    const float mathOffset = mathAngle - calibration.mathReferenceAngle;

    /*
        Apply the actuator diection and then shift result around the actuator reference angle

        Example:
            math reference = 0 deg 
            actuator reference = 90 deg 
            direction = -1

            requested math angle = 20 deg

            actuator angle = 90 + (-1 * (20 - 0)) = 70 deg
    */ 
   return calibration.actuatorReferenceAngle + calibration.direction * mathOffset;
}

JointAngles JointMapper::unmap(const ActuatorTargets& targets) const
{
    JointAngles jointAngles;

    // Reverse the base angle-to-step conversion.
    jointAngles.base = targets.baseSteps / baseStepsPerDegree;

    // Reverse each servo's offset and direction mapping.
    jointAngles.shoulder = unmapServoAngle(
        targets.shoulderServoAngle,
        shoulderCalibration
    );

    jointAngles.elbow = unmapServoAngle(
        targets.elbowServoAngle,
        elbowCalibration
    );

    jointAngles.wristPitch = unmapServoAngle(
        targets.wristPitchServoAngle,
        wristPitchCalibration
    );

    // Wrist roll is not part of ActuatorTargets yet. The caller can
    // fill it from ServoController when reading the current hardware pose.
    jointAngles.wristRoll = 0.0f;

    return jointAngles;
}


float JointMapper::unmapServoAngle(
    float actuatorAngle,
    const JointCalibration& calibration
) const
{
    /*
        Forward mapping:

            actuator = actuatorReference
                       + direction * (math - mathReference)

        Reverse mapping:

            math = mathReference
                   + (actuator - actuatorReference) / direction

        direction is configured as either +1 or -1, so division restores the original mathematical sign.
    */
    return calibration.mathReferenceAngle
        + (actuatorAngle - calibration.actuatorReferenceAngle)
        / calibration.direction;
}
