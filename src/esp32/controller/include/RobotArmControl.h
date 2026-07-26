#pragma once

#include "ForwardKinematics.h"
#include "InverseKinematics.h"
#include "KinematicsData.h"
#include "ServoController.h"
#include "JointMapper.h"

/*
    Coordinates the robot arm's major subsystems.

    This class owns the kinematics solvers and provides one high-level place
    for setup, repeated updates, manual joint commands, and pose calculations.
*/
class RobotArmControl
{
public:
    explicit RobotArmControl(
        const ArmGeometry& geometry,
        const JointCalibration& shoulderCalibration,
        const JointCalibration& elbowCalibration,
        const JointCalibration& wristPitchCalibration,
        float baseStepsPerDegee
    );

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

    /*
        Reads the current physical actuator positions and converts them
        back into mathematical robot-joint angles.
    */
    JointAngles getCurrentJointAngles() const;

    /*
        Calculates the current Cartesian pose from the tracked actuator
        positions. This performs actuator -> mathematical joint mapping
        before running forward kinematics.
    */
    CartesianPose getCurrentPose() const;

    /*
        Solves inverse kinematics and maps the mathematical joint angles into actuator targets
    */
    bool calculateActuatorTargets(const CartesianPose& target, ActuatorTargets& actuatorTargets) const;


    /*
        Prints mathematical joint angles and their mapped actuator targets

        Used for calibration and verification before enabling actuator commands
    */
    bool printMappedPose(const CartesianPose& target) const;


    /*
    Moves the robot arm to a Cartesian target pose

    Returns false if inverse kinematics cannot find a solution

    The pose is first converted into mathematical joint angles, then mapped into physical stepper and servo targets
    */
    bool moveToPose(const CartesianPose& target);

private:
    ForwardKinematics forwardKinematics;
    InverseKinematics inverseKinematics;
    JointMapper jointMapper;
};
