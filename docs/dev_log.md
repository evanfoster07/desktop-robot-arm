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
- Designed, 3D-printed and added a clamp to the arm to stabilize wrist pitch servo

## June 27 2026
- Designed, 3D-printed and attached mount for PCA9685 servo driver

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
- Issue: creep in lazy susan bearing mounts where pegs can no longer hold arm upright at far extents, leading to the arm detaching
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
    - Base rotation control using AccelStepper
    - TMC2209 configuration via UART communication using TMCStepper library
    - PC -> ESP32 Serial commands for testing & controlling motors
- Successfully tested bidirectional base rotation through serial commands

- Next:
    - Configure and test PCA9685 communication over I2C
    - Implement servo control commands for all arm joints
    - Expand serial interface to support individual joint angle commands

## July 15 2026
- Continued development of main servo & stepper control code
- Implemented the following:
    - Servo angle control via Serial commands 
    - `ServoJoint` struct + array architecture for individual servo data organization and accessibility. Includes:
        - Per-servo pulse calibration
        - State tracking
        - Home angles 
    - Improved Serial command parsing to allow testing, calibration and servo/base rotation angle/step selection

- Next:
    - Finish per-servo calibration values using pulse selection Serial commands
    - Fully assemble arm and attempt object manipulation using manual joint position control

## July 16 2026
- Finished per-servo pulse width calibration and implemented as constant values in code 
- Tested object manipulation via manual servo control

- Issue: NEMA 17 stepper motor rotates slightly in base frame during direction changes
    - Fix: secure NEMA 17 using custom 3D printed inserts

- Next:
    - Design & print inserts to secure NEMA 17 stepper motor
    - Begin writing class for inverse/forward kinematics
    - Refactor motor control logic into separate .h and .cpp files

## July 17 2026 
- Refactored motor control and command logic into separate .h and .cpp files 
- Designed and printed inserts to secure NEMA 17 stepper motor
- Tested motor control using refactored code and calibrated arm home position

- Next:
    - Begin writing classes for inverse/forward kinematics
    - Add individual speed control for all servo joints via non-blocking timing logic

## July 18 2026
- Implemented non-blocking per-servo interpolation with configurable movement speeds and continuous state-based servo updates
- Added optional per-servo interpolation, allowing individual joints to move either smoothly or instantly

- Next: 
    - Begin implementing forward kinematics to calculate the end-effector position from joint angles
    - Define arm's link lengths and joint rotation directions in code

## July 19 2026 
- Implemented initial forward kinematics architecture:
    - Created shared kinematics data structures for robot geometry, joint angles, and Cartesian poses
    - Built ForwardKinematics class to calculate gripper pose from joint angles

- Next:
    - Implement inverse kinematics
    - Verify forwards kinematics accuracy with pose calculations from given joint angles

## July 22 2026
- Completed Fusion 360 CAD assembly to verify joint angles & positions for inverse/forward kinematics

- Next:
    - Implement inverse kinematics
    - Verify forwards/inverse kinematics with CAD model and by replicating physical positions
    
## July 24 2026
- Implemented IK math in InverseKinematics class and verified calculations between FK/IK
- Refactored robot arm control into RobotArmControl class:
    - Contains high-level control responsibilities
    - Central class for initialization and update logic
    - Improved organization to prepare for joint mapping and cartesian motion
    - No functional behaviour changes

- Next:
    - Map mathematical IK angles to raw servo command angles
    - Verify raw servo command angle mapping is accurate by testing various cartesian poses

## July 25 2026 
- Finished mapping math IK angles to raw servo commands with JointMapper class and per-servo calibration
- Added unmap() function to reverse mapping operation so FK can be used to obtain cartesian poses from JointAngles
- Implemented poses.h to store cartesian poses for the arm to achieve through serial commands
- Issue:
    - Unmapped physical servo angles give reasonable FK estimate, but when the output cartesian pose is commanded,
    some are calculated as unreachable under servo constraints 
    - Possible fix: 
        - verify constraints are accurate and code from physical angles -> mathematical angles -> FK -> cartesian pose to verify proper calculations
        - Add tolerance to safe servo angles to allow rounding/floating-point differences between FK/IK to achieve similar poses

- Next: 
    - Verify kinematics calculations & test if tolerance fixes pose issue

## July 26 2026
- Traced FK/IK calculations and realized cartestian pose command issue was caused because only the positive IK elbow anlge branch is considered
    - Fixed by adding a negative branch calculation in IK, and selecting branch in calculateBestActuatorTargets() most similar to current arm pose

- Next:
    - Test commands for all predefined grab poses in cartesian form
