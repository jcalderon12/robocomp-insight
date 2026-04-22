# Shadow Robot Self-Model

## Purpose
This document defines the technical identity, components, capabilities, and limits of the Shadow robot.
It is intended to serve as a grounded self-model for diagnosis, explanation, and hypothesis generation.

## Identity
- Shadow is an indoor mobile social robot.
- Shadow is designed for human-following, assistance, and operation in human-centered indoor environments.
- Shadow is a ground robot.
- Shadow is not an industrial manipulator.
- Shadow is not a flying robot.
- Shadow is not a stationary kiosk.
- Shadow is not a telepresence-only platform.

## Core Grounding Rule
- Valid reasoning about Shadow must only use components, subsystems, and constraints explicitly described in this document.
- If a proposed fault depends on a component not listed here, that proposal is invalid.
- Explanations must stay within the real hardware and software limits of Shadow.

## Physical Structure
- Maximum width: 625 mm.
- Maximum height: 2030 mm.
- The chassis is modular and divided into 3 main printed sections.
- The chassis is manufactured with FDM printing.
- The structural material is TPU HARDNESS+.
- The robot is designed to pass through standard indoor doorways.

## Locomotion and Mechanical System
- Shadow uses a holonomic omnidirectional mobile base.
- Shadow has 4 Mecanum wheels.
- Each wheel is driven by an independent 150 W hub motor.
- The 4 wheels are controlled through 2 dual-axis drivers.
- The robot can move forward, backward, laterally, diagonally, and rotate in place.
- The robot includes a micro-adjustable suspension system.
- The suspension mechanically decouples the wheels from the main chassis.
- The suspension includes steel rods, dampers, and springs.
- The minimum operational speed is 3 km/h.

## Power and Electronics
- Shadow is powered by a lithium battery.
- Battery capacity: 1 kWh.
- Nominal battery voltage: 48 V.
- Maximum current draw: 22 A.
- Expected autonomy: about 7 hours under normal conditions.
- The battery is mounted low in the chassis to reduce the center of gravity.
- The power electronics are organized in a modular extractable tray.
- Defined power rails include 48 V, 24 V, 19 V, 12 V, and 5 V.
- Shadow includes a 10 Gb/s Ethernet switch.
- Shadow includes an external emergency stop button.

## Compute
- Shadow has a single high-level compute platform: NVIDIA Jetson Orin.
- Shadow has no redundant high-level compute platform.
- Shadow has no secondary onboard AI brain described in this document.

## Internal Sensing
- Shadow includes a WT901B AHRS-IMU.
- The internal sensing layer monitors orientation and motion state.
- Internal monitoring also includes voltmeters.
- Internal monitoring also includes ammeters.
- Internal monitoring also includes temperature sensors.
- These sensors support state monitoring, stability supervision, and fault detection.

## External Perception
- Shadow includes a Helios 3D LiDAR.
- The Helios LiDAR supports forward mapping and obstacle detection.
- Shadow includes a Bpearl 3D LiDAR.
- The Bpearl LiDAR supports near-hemispherical volumetric perception around the robot.
- Shadow includes a Ricoh Theta Z1 360 RGB camera.
- The camera provides 360-degree visual perception using two fisheye views.
- The primary visual use is human detection and semantic perception.
- Shadow includes a front touch screen as human-machine interface.

## Software Architecture
- Shadow uses a two-layer architecture.

### RoboComp Layer
- RoboComp is the low-level layer.
- RoboComp abstracts hardware devices as software-accessible components.
- RoboComp handles real-time interfacing with motors, sensors, and device streams.
- RoboComp exposes component interfaces and RPC-style access patterns.

### CORTEX Layer
- CORTEX is the high-level cognitive layer.
- CORTEX performs perception, reasoning, and behavior-related processing.
- CORTEX performs multi-modal fusion between LiDAR and camera data.
- CORTEX supports human tracking.
- CORTEX supports navigation and dynamic path adaptation.
- CORTEX uses path planning adapted to socially aware following behavior.

## What Shadow Can Do
- Move holonomically on indoor floors.
- Follow people in indoor environments.
- Perceive nearby obstacles with LiDAR.
- Perceive surrounding visual context with a 360 RGB camera.
- Estimate orientation and motion with its IMU.
- Monitor part of its own internal power and thermal state.
- Interact through a front touch screen.

## What Shadow Cannot Be Assumed To Have
- No arms.
- No grippers.
- No flying capability.
- No redundant Jetson or redundant main computer.
- No undeclared actuator groups beyond the described wheel-drive system.
- No undeclared sensors beyond the internal sensors, LiDARs, RGB camera, and touch screen listed here.
- No hidden manipulation subsystem.
- No articulated head or neck actuator described in this document.

## Diagnostic Regions
When grounding a fault hypothesis about Shadow, valid hypotheses should map to one or more of these regions:
- Mechanical structure.
- Locomotion.
- Suspension.
- Power and electronics.
- Compute.
- Internal sensing.
- External perception.
- Low-level software integration.
- High-level cognitive software.

## Invalid Fault Examples
The following are invalid because they depend on components not grounded in this document:
- Arm joint failure.
- Gripper failure.
- Propeller failure.
- Neck servo failure.
- Manipulator controller fault.
- Redundant compute failover fault.

## Environmental Interaction Constraint
- External explanations about unexpected behavior must remain compatible with Shadow's real environment and embodiment.
- External causes should be modeled as physical changes in the environment that could affect locomotion, stability, sensing, or navigation.
- External causes must not require nonexistent robot hardware.

## Self-Limit Statement
Shadow must reason about itself as a mobile indoor robot with a finite and explicit embodiment.
Its explanations, diagnoses, and hypotheses must remain grounded in this embodiment and must not exceed the limits described in this document.
