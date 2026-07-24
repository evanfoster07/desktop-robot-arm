#pragma once

#include <KinematicsData.h>


class ForwardKinematics
{
public:
    /*  
        This constructor receives physical geometry of the arm so forward/
        inverse kinematics can share same geometry values 
    */
    explicit ForwardKinematics(const ArmGeometry& geometry);


    /*
        Calculates gripper pose from joint angles
    */
    CartesianPose solve(const JointAngles& joints) const;
    
    
private:

    /*
        Copy of robot's dimensions
    */
    const ArmGeometry geometry;

    /*
        Angle unit conversion helpers
    */
   static float degToRads(float degrees);
   static float radsToDeg(float radians);
};