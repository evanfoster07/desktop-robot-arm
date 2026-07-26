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
    return inverseKinematics.solve(target, solution);
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
    JointAngles jointAngles;

    // First convert the cartesian target into mathematical robot joint angles
    const bool reachable = inverseKinematics.solve(target, jointAngles);

    // Return false if pose is geometrically unreachable
    if(!reachable) 
    {
        return false;
    }

    // Will return true if mapped mathematical solution is within servo limits
    return jointMapper.map(jointAngles, actuatorTargets);
}


bool RobotArmControl::printMappedPose(const CartesianPose& target) const
{
    JointAngles jointAngles;
    
    const bool reachable = inverseKinematics.solve(target, jointAngles);

    if (!reachable) 
    {
        Serial.println("Target unreachable.");
        return false;
    }
    ActuatorTargets actuatorTargets;
    const bool actuatorTargetsValid = jointMapper.map(jointAngles, actuatorTargets);

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
    Serial.println(actuatorTargetsValid ? "yes": "no");

    Serial.println();

    return actuatorTargetsValid;
}


bool RobotArmControl::moveToPose(const CartesianPose& target)
{
    JointAngles joints;

    if (!inverseKinematics.solve(target, joints))
    {
        Serial.println("Target unreachable.");
        return false;
    }

    ActuatorTargets targets;

    /*
        Convert to physical actuator commands and verify that every
        mapped servo angle lies inside its safe configured range
    */
    if (!jointMapper.map(joints, targets))
    {
        Serial.println("Target is geometrically reachable, but exceeds safe servo limits.");

        return false;
    }

    // Only move once all targets have been verified
    moveBase(targets.baseSteps - getBasePosition());

    setServoAngle(ServoId::Shoulder, static_cast<int>(std::round(targets.shoulderServoAngle)));
    setServoAngle(ServoId::Elbow, static_cast<int>(std::round(targets.elbowServoAngle)));
    setServoAngle(ServoId::WristPitch, static_cast<int>(std::round(targets.wristPitchServoAngle)));

    return true;
}