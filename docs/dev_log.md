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
- Traced FK/IK calculations and realized cartestian pose command issue was caused because only the positive IK elbow angle branch is considered
    - Fixed by adding a negative branch calculation in IK, and selecting branch in calculateBestActuatorTargets() most similar to current arm pose

- Next:
    - Test commands for all predefined grab poses in cartesian form

## July 27 2026
- Tested and verified function for all grab pose commands in cartesian form
- Carried out test pick-and-place tasks with various objects and pre-defined cartesian poses

- Next:
    - Learn about integrating a Raspberry Pi 5 + camera for object recognition
    - Script short demo video for current state of project, showcasing:
        - FK/IK
        - Pick-and-place tasks
        - Dev process so far
        - Kinematics math

## July 28 2026
- Issue: Found that UART TX/RX pins were connected to the wrong TMC2209 pin, meaning UART configuration was not being applied
    - Fix: re-solder ESP32 -> TMC2209 UART wires onto proper pins

## === BREAK (trip) ===


## August 16 2026
- Fixed UART RX/TX pin connection issue and verified UART communication
- Tested various microstepping and RMS current settings as well as driver modes

- Issue: NEMA17 coupler slips under arm load approximately 22° (200 microsteps = approx 22°)
    - Fix: tighten coupling shaft tolerance and re-print part

## August 17 2026
- Tightened CAD tolerance for NEMA17 -> base coupler, then re-printed and attached finished part
- Designed, printed and attached Raspberry pi camera mount + module to arm

- Issue: tested new motor hub, however slip is still present for ~4° of rotation 
    - Fix: re-design hub with tapered D-bore to minimize slip

- Next: print and attach new NEMA17 hub and begin planning CV implementation with R+aspberry pi 

## August 18 2026
- Re-designed, then printed and attached NEMA17 motor hub with internal tapered D-bore and external taper for tighter base-platform coupling
    - Reduced overall slip to ~0.2° from ~22° after testing (2 microsteps slip = approx 0.2°)
- Configured and tested Raspberry pi SSH for CV development

- Next:
    - Begin CV development on Pi
    - Establish Pi -> ESP32 communication via Serial USB connection

## August 20 2026
- Began CV testing with OpenCV ORB feature detection
    - Successfully detected and matched features between reference and scene images
    - Homography-based object localization was unreliable, particularly with changes in viewing angle
    - Decided to move toward CNN-based object detection using transfer learning
    
- Next:
    - Begin planning CNN code architecture
    - Obtain & label training images from robot perspective before camera extension arrives

## August 22 2026
- Began YOLO object detection implementation
    - Installed Ultralytics YOLO and PyTorch on Raspberry Pi
    - Loaded pretrained YOLO26n model
    - Integrated OpenCV image loading with YOLO inference
    - Successfully detected objects from test images
        - Tested detection on target creeper keychain object

- Next:
    - Create custom creeper keychain dataset
    - Label training images with bounding boxes

## August 23 2026
- Implemented custom YOLO object detection pipeline
    - Created and annotated creeper dataset
    - Created Python script to convert Ultralytics NDJSON annotations to YOLO train/val format
    - Tested fine-tuning of pretrained YOLO26n model on Raspberry Pi
    - Evaluated model using precision, recall, mAP, and loss
    - Initial model showed poor generalization due to very limited training data

- Next:
    - Expand dataset
    - Train same dataset on laptop with frozen early layers to compare performance
    - Integrate Pi camera with OpenCV inference

## August 24 2026
- Moved YOLO training from Raspberry Pi to laptop for significantly faster training
- Retrained test creeper detection model on laptop for 20 epochs
- Tested trained model on new images and training set images
    - For new images: extremely low confidence, excessive detections
    - Training set images: higher but still low confidence, more reliable detections
    
- Next:
    - Continue collecting diverse training images with Raspberry Pi camera
    - Improve dataset/model performance before deploying inference to Raspberry Pi

## August 25 2026
- Integrated IMX219 eye-in-hand camera with Raspberry Pi 5
    - Connected camera using extended CSI ribbon cable setup for arm mobility
    - Diagnosed camera detection and ribbon connection issues
    - Configured IMX219 device tree overlay for camera detection
- Verified camera is successfully detected by Raspberry Pi

- Next:
    - Develop automated image capture and naming script
    - Collect robot-perspective images for YOLO dataset
    - Label images and retrain object detection model