You explain anomalies of a mobile robot. You propose hypotheses; the robot's memory checks them against the recording and simulates the ones that survive.

## The episode rec_mission_Follow_Person_06102026_093231

Room frame: x and y in metres, headings in degrees (0 = +x, counter-clockwise). Times in seconds from the start of the recording.

### What happened
- The robot was following a person with a bottle on its tray. Support_bottle: the robot held the bottle from 0 s to 14.955 s.
- Accident_1: the bottle was lost at t_obs = 14.955 s, with the robot at (0, 0), heading 90 deg, on Segment_final. Its cause is unknown.
- After t_obs the robot reacted to the loss (its speed setpoint went to 0 at 15.072 s). That reaction is a consequence, not a cause: it is left out below.

### Phases of the motion before the fall
- Phase_1 (stopped), Interval_Phase_1 = [2.546, 14.955] s: mean speed 0.242 m/s, commanded 0-0 m/s, heading from 90 to 90 deg.

### Path segments, counted back from the fall (the anchors for a place: `segment`)
- Segment_final: the last 1 m of the path before the fall, from (-1, 0) to (0, 0), heading 0 deg, mean speed 0.112 m/s, driven during Interval_Segment_final = [6.064, 14.955] s.
- Segment_prev_1: the path between 1 and 2 m before the fall, from (-2, 0) to (-1, 0), heading 0 deg, mean speed 0.288 m/s, driven during Interval_Segment_prev_1 = [2.593, 6.064] s.
- Segment_prev_2: the path between 2 and 3 m before the fall, from (-3, 0) to (-2, 0), heading 0 deg, mean speed 21.034 m/s, driven during Interval_Segment_prev_2 = [2.546, 2.593] s.

### Time intervals (the anchors for a time: `interval`)
- Interval_fall = [0.546, 14.955] s: the last 14.4 s before the fall.
- Interval_Phase_1 = [2.546, 14.955] s: the whole of Phase_1 (stopped).
- Interval_Segment_final = [6.064, 14.955] s: the drive over the last 1 m of the path before the fall.
- Interval_Segment_prev_1 = [2.593, 6.064] s: the drive over the path between 1 and 2 m before the fall.
- Interval_Segment_prev_2 = [2.546, 2.593] s: the drive over the path between 2 and 3 m before the fall.

### Evidence (what the recording shows; "in free motion" is the same quantity over Interval_baseline)
- pitch_rate_peak = 1 rad/s at 14.48 s, over Interval_fall. (pitch rate peak: Max |gyroscope x| in the interval (rad/s).)
- vertical_accel_peak = 2.81 m/s2 at 12.848 s, over Interval_fall. (vertical acceleration peak: Max |accelerometer z - g| in the interval (m/s2).)
- longitudinal_accel_peak = 3 m/s2 at 12.128 s, over Interval_fall. (longitudinal acceleration peak: Max |accelerometer y| (forward axis) in the interval (m/s2).)
- lateral_accel_peak = 1 m/s2 at 13.776 s, over Interval_fall. (lateral acceleration peak: Max |accelerometer x| in the interval (m/s2).)
- yaw_change = 0 deg, over Interval_fall; commanded: 0.92 deg. (yaw change: Heading change of the recorded pose over the interval, next to the commanded one: -integral of robot_ref_rot_speed, whose sign is opposite to the executed turn (deg).)
- yaw_rate_peak = 0 rad/s at 0.56 s, over Interval_fall. (yaw rate peak: Max |gyroscope z| in the interval (rad/s).)
- setpoint_step = 0 m/s, over Interval_fall. (setpoint step: Largest change of the filtered forward setpoint (0.2 s moving median of the zero-order hold) within 0.5 s (m/s).)
- speed_ratio = None, over Interval_fall. (speed ratio: Path / integral of the setpoint in the interval, divided by the same ratio in Interval_baseline. Relative to the episode itself because of the Webots clock.)
- max_sustained_horizontal_accel = 1.414 m/s2 at 13.648 s, over Interval_Phase_1. (max sustained horizontal acceleration: Max before t_obs of the min of 3 consecutive horizontal acceleration samples (m/s2). Measured in simulated time, so independent of the Webots clock.)
- person_distance_min = 3 m, over Interval_fall. (minimum person distance: Minimum robot-person distance in the interval (m).)
- bottle_reacquired = no. (bottle reacquired: Whether the robot->bottle RT edge appears again after t_obs.)

## The mechanisms

The closed list of ways the bottle can fall. Use their ids. Their parameters are kinds only: how big, how strong or how slippery is not chosen, because the memory simulates the whole range it can.

- obstacle_traversed: The robot drives over a low object on the floor (a bump or a cable) that it did not detect; the jolt tips the bottle.
  Anchors: segment. Parameters: shape: bump | cable.
  Precondition: Mean speed on the anchored segment > 0.05 m/s.
  Leaves a trace on: pitch_rate_peak, vertical_accel_peak.
  Simulable: yes. Cost: a bump, 4 simulations (one per bump size); a cable, 1.
- bottle_push: Something (the followed person, an object, a gust) pushes the bottle off the tray; the robot itself feels no jolt.
  Anchors: interval. Parameters: direction: any | forward | backward | left | right.
  Precondition: The support of the bottle covers the start of the anchored interval.
  Leaves a trace on: nothing the recording measures.
  Simulable: yes. Cost: 1 simulation.
- robot_push: Something pushes the robot; the jolt or the deviation tips the bottle.
  Anchors: interval. Parameters: direction: any | forward | backward | left | right.
  Precondition: none.
  Leaves a trace on: lateral_accel_peak, speed_ratio, yaw_change.
  Simulable: yes. Cost: 1 simulation.
- slippery_floor: The whole floor is slippery; the wheels slip and the bottle is displaced.
  Anchors: none. Parameters: none.
  Precondition: none.
  Leaves a trace on: max_sustained_horizontal_accel, speed_ratio.
  Simulable: yes. Cost: 1 simulation.
- wheel_failure: One of the two drive wheels stops and stays braked; the robot turns without being ordered to.
  Anchors: interval. Parameters: side: left | right | unknown.
  Precondition: Mean speed in the anchored interval > 0.05 m/s.
  Leaves a trace on: speed_ratio, yaw_change, yaw_rate_peak.
  Simulable: yes. Cost: 1 per side; side unknown, 2.
- commanded_speed_change: The robot itself was ordered to brake or accelerate sharply; the order is in the recorded setpoint.
  Anchors: interval. Parameters: none.
  Precondition: none.
  Leaves a trace on: setpoint_step.
  Simulable: the nominal run, which replays the recorded setpoints, already reproduces it. Cost: none (not simulated).
- uncommanded_motion: The base did not do what it was ordered: a brake or a jerk that is not in the setpoint (driver or power fault).
  Anchors: interval. Parameters: kind: brake | jerk.
  Precondition: Mean speed in the anchored interval > 0.05 m/s.
  Leaves a trace on: longitudinal_accel_peak, speed_ratio.
  Simulable: not yet: its simulation is experimental. Cost: none (not simulated).
- bottle_removed_by_person: A person takes the bottle off the tray; there is no jolt and the person is within reach.
  Anchors: interval. Parameters: none.
  Precondition: none.
  Leaves a trace on: person_distance_min.
  Simulable: no. Not physical: it is checked against the recording, not simulated. Cost: none (not simulated).
- perception_failure: The bottle did not fall: perception lost it; there is no jolt and the bottle is detected again.
  Anchors: interval. Parameters: none.
  Precondition: none.
  Leaves a trace on: bottle_reacquired.
  Simulable: no. Not physical: the bottle did not fall; it is checked against the recording. Cost: none (not simulated).
- suspension_failure: The suspension fails or vibrates and shakes the bottle off.
  Anchors: none. Parameters: none.
  Precondition: none.
  Leaves a trace on: nothing the recording measures.
  Simulable: no. The PyBullet URDF has no suspension. Cost: none (not simulated).
- new: a mechanism that is not in this list. Describe it in new_mechanism_description; its anchors are optional. It is kept to extend the list, not simulated.

## The robot (its self-model)

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
- Shadow uses a differential-drive mobile base; it is not holonomic.
- Shadow has 2 drive wheels on a common lateral axis and 2 passive caster wheels, one at the front and one at the back.
- The drive wheels have a radius of 100 mm and are 518 mm apart.
- Each drive wheel has its own hub motor; both motors are controlled by a single dual-axis SVD48V driver.
- The robot can move forward and backward, turn while moving, and rotate in place; it cannot move sideways.
- The base limits linear speed to 0.9 m/s and rotation speed to 2 rad/s, accelerates at up to 0.5 m/s^2 and brakes at up to 1.0 m/s^2.
- The robot includes a micro-adjustable suspension system.
- The suspension mechanically decouples the wheels from the main chassis.
- The suspension includes steel rods, dampers, and springs.

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
- Drive on indoor floors with differential steering (forward, backward, turning, and rotating in place).
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

## Your task

The bottle fell from the tray and the robot perceived no cause. Propose the mechanisms that could explain the fall, as hypotheses written in symbols:
- each hypothesis names one mechanism id from the list (or `new`), the anchors that mechanism takes (segment and interval ids from the episode), and its qualitative parameters. Every parameter of the mechanism is required, with one of its listed values;
- never write a number: no coordinates, times, fractions, forces nor probabilities. The memory turns the symbols into numbers;
- read the evidence: a hypothesis should explain what the recording shows;
- order the hypotheses from the most to the least plausible; propose at most 10. Each one is checked against the recording, and up to 6 simulations are run, in your order, with the ones that survive. Each hypothesis takes the cost of its mechanism, all of it or nothing: the first one that does not fit in what is left ends the simulations;
- propose a mechanism that explains the evidence even if it cannot be simulated;
- the bottle leaving the tray is the effect, not a cause, and the robot's reaction after t_obs is not a cause either.

Answer with exactly one JSON object and nothing else (no markdown, no prose), in this format:
{
  "hypotheses": [
    {
      "mechanism": "<a mechanism id, or new>",
      "new_mechanism_description": "<only with new; otherwise null>",
      "segment": "<a segment id, or null>",
      "interval": "<an interval id, or null>",
      "qualitative_parameters": {
        "<parameter>": "<one of its values>"
      },
      "expected_trace": [
        "<what the recording should show if this is the cause>"
      ],
      "rationale": "<why, from the evidence>",
      "title": "<a short title>"
    }
  ]
}
