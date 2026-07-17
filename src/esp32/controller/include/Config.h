#pragma once

#include <Arduino.h>

namespace Pins {
    
    //Servo I2C pins
    constexpr int SCL_PIN = 22;
    constexpr int SDA_PIN = 21;

    //TMC2209 & Stepper control pins
    constexpr int STEPPER_EN = 25;
    constexpr int UART_RX = 16;
    constexpr int UART_TX = 17;
    constexpr int STEP_PIN = 32;
    constexpr int DIR_PIN = 33;
}

namespace StepperConfig {

    //TMC2209 sense resistor value
    constexpr float R_SENSE = 0.11f;

    //TMC2209 UART address
    constexpr uint8_t DRIVER_ADDRESS = 0;
    constexpr uint32_t UART_BAUD_RATE = 115200;

    //TMC2209 config
    constexpr uint16_t RMS_CURRENT_MA = 500;
    constexpr uint16_t MICROSTEPS = 16;

    constexpr float MAX_SPEED = 800.0f;
    constexpr float ACCELERATION = 300.0f;
}

namespace ServoConfig {

    //PCA9685 address & PWM frequency
    constexpr uint8_t PCA9685_ADDRESS = 0x40;
    constexpr int PWM_FREQUENCY = 50;     // 50 Hz
}