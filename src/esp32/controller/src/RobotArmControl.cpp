#include "RobotArmControl.h"

#include "StepperController.h"

RobotArmControl::RobotArmControl(
    const ArmGeometry& geometry,
    const JointCalibration& shoulderCalibration,
    const JointCalibration& elbowCalibration,
    const JointCalibration& wristPitchCalibration,
    float baseStepsPerDegree
)
    : forwardKinematics(geometry),
      inverseKinematics(geometry),
      jointMapper(
          shoulderCalibration,
          elbowCalibration,
          wristPitchCalibration,
          baseStepsPerDegree
      )
{
}

void RobotArmControl::begin()
{
    beginServos();
    beginStepper();
}

void RobotArmControl::update()
{
    updateStepper();
    updateServos();
}

void RobotArmControl::setServoAngle(ServoId servoId, int requestedAngle)
{
    ::setServoAngle(servoId, requestedAngle);
}

void RobotArmControl::writeServoPulse(ServoId servoId, int pulseWidthUS)
{
    ::writeServoPulse(servoId, pulseWidthUS);
}

void RobotArmControl::homeServos()
{
    ::homeServos();
}

void RobotArmControl::moveBase(long relativeSteps)
{
    ::moveBase(relativeSteps);
}

void RobotArmControl::stopBase()
{
    ::stopBase();
}

long RobotArmControl::getBasePosition() const
{
    return ::getBasePosition();
}

bool RobotArmControl::solveInverseKinematics(const CartesianPose& target, JointAngles& solution) const
{
    JointAngles positiveBranch;
    JointAngles negativeBranch;

    const bool positiveReachable = inverseKinematics.solve(target, positiveBranch, true);
    const bool negativeReachable = inverseKinematics.solve(target, negativeBranch, false);

    if (!positiveReachable && !negativeReachable)
    {
        return false;
    }

    if (positiveReachable && !negativeReachable)
    {
        solution = positiveBranch;
        return true;
    }

    if (!positiveReachable && negativeReachable)
    {
        solution = negativeBranch;
        return true;
    }

    const JointAngles referenceJoints = getCurrentJointAngles();

    const float positiveScore =
        (positiveBranch.shoulder - referenceJoints.shoulder) * (positiveBranch.shoulder - referenceJoints.shoulder) +
        (positiveBranch.elbow - referenceJoints.elbow) * (positiveBranch.elbow - referenceJoints.elbow) +
        (positiveBranch.wristPitch - referenceJoints.wristPitch) * (positiveBranch.wristPitch - referenceJoints.wristPitch);

    const float negativeScore =
        (negativeBranch.shoulder - referenceJoints.shoulder) * (negativeBranch.shoulder - referenceJoints.shoulder) +
        (negativeBranch.elbow - referenceJoints.elbow) * (negativeBranch.elbow - referenceJoints.elbow) +
        (negativeBranch.wristPitch - referenceJoints.wristPitch) * (negativeBranch.wristPitch - referenceJoints.wristPitch);

    solution = (positiveScore <= negativeScore) ? positiveBranch : negativeBranch;

    return true;
}

CartesianPose RobotArmControl::solveForwardKinematics(const JointAngles& joints) const
{
    return forwardKinematics.solve(joints);
}

JointAngles RobotArmControl::getCurrentJointAngles() const
{
    /*
        ServoController tracks physical servo angles. JointMapper expects
        those physical values in ActuatorTargets so it can reverse the
        calibration offsets and direction signs.
    */
    ActuatorTargets currentTargets;

    currentTargets.baseSteps = getBasePosition();
    currentTargets.shoulderServoAngle =
        getServoJoint(ServoId::Shoulder).currAngle;
    currentTargets.elbowServoAngle =
        getServoJoint(ServoId::Elbow).currAngle;
    currentTargets.wristPitchServoAngle =
        getServoJoint(ServoId::WristPitch).currAngle;

    JointAngles joints = jointMapper.unmap(currentTargets);

    /*
        Wrist roll is currently not mapped by JointMapper because it is
        not used by the planar IK calculation. Its servo angle directly
        represents the reported roll for now.
    */
    joints.wristRoll = getServoJoint(ServoId::WristRoll).currAngle;

    return joints;
}


CartesianPose RobotArmControl::getCurrentPose() const
{
    /*
        ForwardKinematics must receive mathematical joint angles, not
        raw servo angles. getCurrentJointAngles() performs that reverse
        conversion first.
    */
    return forwardKinematics.solve(getCurrentJointAngles());
}

bool RobotArmControl::calculateActuatorTargets(const CartesianPose& target, ActuatorTargets& actuatorTargets) const 
{
    const JointAngles referenceJoints = getCurrentJointAngles();
    JointAngles jointAngles;

    return calculateBestActuatorTargets(target, referenceJoints, jointAngles, actuatorTargets);
}


bool RobotArmControl::calculateActuatorTargets(
    const CartesianPose& target,
    const JointAngles& referenceJoints,
    JointAngles& solution,
    ActuatorTargets& actuatorTargets
) const
{
    return calculateBestActuatorTargets(target, referenceJoints, solution, actuatorTargets);
}


bool RobotArmControl::calculateBestActuatorTargets(
    const CartesianPose& target,
    const JointAngles& referenceJoints,
    JointAngles& solution,
    ActuatorTargets& actuatorTargets
) const
{
    JointAngles positiveBranch;
    JointAngles negativeBranch;
    ActuatorTargets positiveTargets;
    ActuatorTargets negativeTargets;

    const bool positiveReachable = inverseKinematics.solve(target, positiveBranch, true);
    const bool negativeReachable = inverseKinematics.solve(target, negativeBranch, false);

    if (!positiveReachable && !negativeReachable)
    {
        return false;
    }

    const bool positiveValid = positiveReachable && jointMapper.map(positiveBranch, positiveTargets);
    const bool negativeValid = negativeReachable && jointMapper.map(negativeBranch, negativeTargets);

    if (!positiveValid && !negativeValid)
    {
        return false;
    }

    // Compare 'score' of each branch for best option
    const float positiveScore =
        (positiveBranch.shoulder - referenceJoints.shoulder) * (positiveBranch.shoulder - referenceJoints.shoulder) +
        (positiveBranch.elbow - referenceJoints.elbow) * (positiveBranch.elbow - referenceJoints.elbow) +
        (positiveBranch.wristPitch - referenceJoints.wristPitch) * (positiveBranch.wristPitch - referenceJoints.wristPitch);

    const float negativeScore =
        (negativeBranch.shoulder - referenceJoints.shoulder) * (negativeBranch.shoulder - referenceJoints.shoulder) +
        (negativeBranch.elbow - referenceJoints.elbow) * (negativeBranch.elbow - referenceJoints.elbow) +
        (negativeBranch.wristPitch - referenceJoints.wristPitch) * (negativeBranch.wristPitch - referenceJoints.wristPitch);

    if (positiveValid && (!negativeValid || positiveScore <= negativeScore))
    {
        solution = positiveBranch;
        actuatorTargets = positiveTargets;
        return true;
    }

    solution = negativeBranch;
    actuatorTargets = negativeTargets;
    return true;
}


bool RobotArmControl::printMappedPose(const CartesianPose& target) const
{
    const JointAngles referenceJoints = getCurrentJointAngles();
    JointAngles jointAngles;
    ActuatorTargets actuatorTargets;

    if (!calculateBestActuatorTargets(target, referenceJoints, jointAngles, actuatorTargets))
    {
        Serial.println("Target unreachable.");
        return false;
    }

    Serial.println();
    Serial.println("Mathematical joint angles:");

    Serial.print("Base: ");
    Serial.println(jointAngles.base);

    Serial.print("Shoulder: ");
    Serial.println(jointAngles.shoulder);

    Serial.print("Elbow: ");
    Serial.println(jointAngles.elbow);

    Serial.print("Wrist pitch: ");
    Serial.println(jointAngles.wristPitch);


    Serial.println();
    Serial.println("Mapped actuator targets:");

    Serial.print("Base steps: ");
    Serial.println(actuatorTargets.baseSteps);

    Serial.print("Shoulder servo: ");
    Serial.println(actuatorTargets.shoulderServoAngle);

    Serial.print("Elbow servo: ");
    Serial.println(actuatorTargets.elbowServoAngle);

    Serial.print("Wrist-pitch servo: ");
    Serial.println(actuatorTargets.wristPitchServoAngle);

    Serial.println();

    Serial.print("Physical pose valid: "); 
    Serial.println("yes");

    Serial.println();

    return true;
}


bool RobotArmControl::moveToPose(const CartesianPose& target)
{
    const JointAngles referenceJoints = getCurrentJointAngles();
    JointAngles joints;
    ActuatorTargets targets;

    // Do not move if both pos & neg targets are unreachable 
    if (!calculateBestActuatorTargets(target, referenceJoints, joints, targets))
    {
        Serial.println("Target unreachable.");
        return false;
    }

    // Only move once all targets have been verified
    moveBase(targets.baseSteps - getBasePosition());

    setServoAngle(ServoId::Shoulder, static_cast<int>(std::round(targets.shoulderServoAngle)));
    setServoAngle(ServoId::Elbow, static_cast<int>(std::round(targets.elbowServoAngle)));
    setServoAngle(ServoId::WristPitch, static_cast<int>(std::round(targets.wristPitchServoAngle)));

    return true;
}