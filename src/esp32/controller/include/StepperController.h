#pragma once

#include <Arduino.h>

/*
    Initializes:
    - Serial2 UART
    - TMC2209 configuration
    - AccelStepper motion settings
*/
void beginStepper();


/*
    Adds a relative movement to the stepper's target position.

    Positive and negative values for opposite directions.
*/
void moveBase(long relativeSteps);


/*
    Controlled stop using AccelStepper's deceleration.
*/
void stopBase();


/*
    Must be called repeatedly from loop().
*/
void updateStepper();


/*
    Returns AccelStepper's current position in steps.
*/
long getBasePosition();