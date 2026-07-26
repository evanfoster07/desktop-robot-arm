#pragma once

#include "KinematicsData.h"
#include <Arduino.h>

/*
    Collection of predefined Cartesian poses that the robotcan move to.
*/

constexpr CartesianPose POSES[]
{

    // Home
    {
        22.63f,
        0.00f,
        287.45f,
        10.00f,
        90.0f
    },


    // Grab Pose 1
    {
        113.31f,
        196.86f,
        59.58f,
        -60.00f,
        90.0f
    },

    // Grab Pose 2
    {
        1.79f,
        3.10f,
        278.76f,
        15.00f,
        90.0f
    },

    // Grab Pose 3
    {
        1.79f,
        3.10f,
        278.76f,
        15.00f,
        90.0f
    },

    

};

constexpr size_t POSE_COUNT = sizeof(POSES) / sizeof(POSES[0]);