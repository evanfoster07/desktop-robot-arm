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


    // Grab Pose 1 (Pickup, ground)
    {
        113.31f,
        196.86f,
        59.58f,
        -60.00f,
        90.0f
    },

    // Grab Pose 2 (Intermediate, rotate to pose 3)
    {
        -2.18f,
        2.84f,
        278.76f,
        15.00f,
        90.0f
    },

    // Grab Pose 3 (Destination, ground)
    {
        -97.09f,
        126.19f,
        48.83f,
        -95.00f,
        90.0f
    },

    // Grab pose 4 (Destination, PSU Height)
    {
        -185.27f,
        189.68f,
        229.10f,
        5.00f,
        90.0f
    }
};

constexpr size_t POSE_COUNT = sizeof(POSES) / sizeof(POSES[0]);