#pragma once 

#include "KinematicsData.h"


class InverseKinematics
{
public:

    /*
        This constructor receives physical geometry of the arm so forward/
        inverse kinematics can share same geometry values 
    */
    explicit InverseKinematics(const ArmGeometry& geometry);

    /*
        Calculates the joint angles needed to reach a desired Cartesian position

        target:
            the desired Cartesian position

        solution:
            the joint angles required to reach the solution

        Return value:
            true = target can be reached 
            false = target cannot be reached
    */
    bool solve(
        const CartesianPose& target,
        JointAngles& solution,
        bool preferElbowPositive = true
    ) const;


private:
    /*
        Copy of robot's physical dimensions
    */
    const ArmGeometry geometry;

    /*
        Angle unit conversion helpers
    */
    static float degToRads(float degrees);
    static float radsToDeg(float radians);
};


