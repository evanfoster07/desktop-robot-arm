#include "RobotArmControl.h"

#include "StepperController.h"

RobotArmControl::RobotArmControl(const ArmGeometry& geometry)
    : forwardKinematics(geometry),
      inverseKinematics(geometry)
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
