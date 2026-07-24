#pragma once

#include "ForwardKinematics.h"
#include "InverseKinematics.h"
#include "KinematicsData.h"
#include "ServoController.h"

/*
    Coordinates the robot arm's major subsystems.

    This class owns the kinematics solvers and provides one high-level place
    for setup, repeated updates, manual joint commands, and pose calculations.
*/
class RobotArmControl
{
public:
    explicit RobotArmControl(const ArmGeometry& geometry);

    /*
        Initializes the servo and stepper hardware.
    */
    void begin();

    /*
        Advances every non-blocking actuator controller.

        Must be called repeatedly from loop().
    */
    void update();

    /*
        Commands a raw physical servo angle.
    */
    void setServoAngle(ServoId servoId, int requestedAngle);

    /*
        Writes a raw pulse width for servo calibration.
    */
    void writeServoPulse(ServoId servoId, int pulseWidthUS);

    /*
        Moves all servos to their configured home angles.
    */
    void homeServos();

    /*
        Adds relative motion to the base stepper target.
    */
    void moveBase(long relativeSteps);

    /*
        Requests a controlled base stop.
    */
    void stopBase();

    /*
        Returns the base stepper's current position in steps.
    */
    long getBasePosition() const;

    /*
        Solves the mathematical joint angles for a Cartesian target.

        This only calculates the solution. It does not move the hardware yet.
    */
    bool solveInverseKinematics(const CartesianPose& target, JointAngles& solution) const;

    /*
        Calculates the Cartesian pose produced by mathematical joint angles.
    */
    CartesianPose solveForwardKinematics(const JointAngles& joints) const;

private:
    ForwardKinematics forwardKinematics;
    InverseKinematics inverseKinematics;
};
