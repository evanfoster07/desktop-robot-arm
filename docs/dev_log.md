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

## July 4 2026
- Finished planning component/connector placements 
- Organized components on perfboards to prepare for soldering 
- Next: 
    - Solder components and wire connections on perfboards 
    - Crimp signal/power wires with JST XH connectors and shrink wrap together for organization

## July 6 2026
- cut, organized and crimped with JST XH connectors the required wires for internal power/signal connections
- Next:
    - Begin soldering header pins onto perfboard 
    - Solder wire connections between header pins on perfboard 

## July 8 2026
- Completed soldering all female/male dupont header pins, JST XH female connectors, and power screw terminals onto perfboards
- Began soldering 6V and GND power rails and routed to JST XH female pins for power distribution
- Tested solder joints and performed continuity tests on completed perfboard connections
- Next: 
    - Continue soldering wire connections on perfboard

## July 10 2026
- Finished soldering wire connections for both servo driver and stepper driver communication lines and power on perfboards
- Continuity tested all connections and ensured common ground between all components
- Verified no short circuits were present in wiring
- Next:
    - Attach finished perfboards to base of arm and verify function with test code
    - Begin writing integrated control code for stepper & servo motors

## July 11 2026
- Mounted all perfboards to base of arm and attached all JST/Dupont cables 
- Performed continuity testing between all major components' pins and checked for short circuits
- Next: 
    - Begin writing integrated control code for all motor control

## July 14 2026
- Began writing main servo & stepper control code
- Implemented the following:
    - Base rotation test code 
    - TMC2209 configuration via UART communication using TMCstepper library
    - PC -> ESP32 Serial commands for testing & controlling motors
- Next:
    - Implement control for servos using I2C pins and Adafruit PWM Servo Driver library to communicate with PCA9685