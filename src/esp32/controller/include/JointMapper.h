#pragma once

#include "KinematicsData.h"


/*
    Converts mathematical robot-joint angles into physical
    actuator commands

    JointMapper applies:
        - reference-angle offsets
        - direction inversions
        - base angle-to-step conversion
*/
class JointMapper
{
public:

    /*
        Creates a mapper using the calibration data for each joint

        stepsPerDegree:
            Number of stepper motor steps required for one degree
            of base rotation
    */
    JointMapper(
        const JointCalibration& shoulderCalibration,
        const JointCalibration& elbowCalibration,
        const JointCalibration& wristPitchCalibration,
        float baseStepsPerDegree
    );


    /*
        Converts mathematical joint angles into actuator targets

        Returns true when all mapped servo angles lie inside their
        configured safe ranges

        Returns false when at least one mapped servo angle is invalid
    */
    bool map(const JointAngles& jointAngles, ActuatorTargets& targets) const;


    /*
        Converts physical actuator targets back into mathematical
        robot-joint angles.

        This is the reverse of map() and is used before forward
        kinematics when the starting values came from the hardware.
    */
    JointAngles unmap(const ActuatorTargets& targets) const;


private:

    JointCalibration shoulderCalibration;
    JointCalibration elbowCalibration;
    JointCalibration wristPitchCalibration;

    float baseStepsPerDegree;


    /*
        Applies one joint's reference offset and direction.
    */
    float mapServoAngle(
        float mathAngle,
        const JointCalibration& calibration
    ) const;


    /*
        Reverses one servo's reference offset and direction mapping.
    */
    float unmapServoAngle(
        float actuatorAngle,
        const JointCalibration& calibration
    ) const;
};