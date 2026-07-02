## June 9 -> June 23 2026
- Modelled and 3D-printed the following: 
    - Base frame 
    - Upper & lower arm link and pitch joints 
    - Wrist link and pitch joint 
    - Wrist roll joint
    - Gripper open/close mechanism
- Assembled main components and servos
- Tested all servo function and ensured sufficient shoulder torque for full 180° ROM
- Created code to test the control of each servo angle individually and tested combined rotation sequences

## June 25 2026
- Tested PCA9685 servo driver using Adafruit servo driver library examples
- Configured PCA9685 and calibrated min/max pulse width values for accurate positioning
- Desgined, 3D-printed and added a clamp to the arm to stabalize wrist pitch servo

## June 27 2026
- Designed, 3D-print and attached mount for PCA9685 servo driver
- Next: attach M-F jumper wires to lower arm for wrist roll + gripper servo wire extension 

## June 29 2026
- Attached M-F jumper wires to extend wrist roll + gripper servo wire connections
- Tested PCA9685 servo driver and servo function with new mount + wire extensions
- Next: 
    - Set up TMC2209 stepper motor driver and test base rotation 
    - Start electrical schematic and begin planning internal component placement

## June 30 2026
- Began electrical schematic and created custom symbols for ESP32, TMC2209, and PCA9685
- Started wiring TMC2209 stepper motor driver to test base rotation & driver control 
- Next:
    - Finish electrical schematic
    - Write TMC2209 control code and test stepper motor base rotation

## July 1 2026
- Finished wiring TMC2209 stepper motor driver and tested with control code
- Next:
    - Finish electrical schematic
    - Begin planning internal wire pathing, component locations on perfboards, and communication/power terminal locations

## July 2 2026
- Finished electrical schematic primary power rails/distribution & communication connections 
- Began planning electrical component and connector placements in Fusion 360 assembly model
- Issue: creep in lazy susan bearing mounts where pegs can no longer hold arm upright at far extents, leading to the arm deattaching
    - Will fix with custom threaded pegs and nuts to secure in place
- Next:
    - Continue planning component/connector placements for internal wiring
    - Print and attach new lazy Susan mounts and test at max arm extent to verify mount stability at maximum torque


