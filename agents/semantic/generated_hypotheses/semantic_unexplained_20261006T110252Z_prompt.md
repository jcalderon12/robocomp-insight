You explain anomalies of a mobile robot. You propose hypotheses; the robot's memory checks them against the recording and simulates the ones that survive.

## The episode rec_mission_Follow_Person_06102026_130107

Room frame: x and y in metres, headings in degrees (0 = +x, counter-clockwise). Times in seconds from the start of the recording.

### What happened
- The robot was following a person with a bottle on its tray. Support_bottle: the robot held the bottle from 0 s to 13.6 s.
- Accident_1: the bottle was lost at t_obs = 13.6 s, with the robot at (-0.169, -0.39), heading 16.4 deg, on Segment_final. Its cause is unknown.
- After t_obs the robot reacted to the loss (its speed setpoint went to 0 at 13.712 s). That reaction is a consequence, not a cause: it is left out below.

### Phases of the motion before the fall
- Phase_1 (turn), Interval_Phase_1 = [0.24, 1.696] s: mean speed 0.302 m/s, commanded 0.54-0.59 m/s, heading from -14.9 to -7.5 deg.
- Phase_2 (advance_straight), Interval_Phase_2 = [1.696, 13.6] s: mean speed 0.261 m/s, commanded 0.32-0.57 m/s, heading from -7.5 to 16.8 deg.

### Path segments, counted back from the fall (the anchors for a place: `segment`)
- Segment_final: the last 1 m of the path before the fall, from (-1.16, -0.423) to (-0.169, -0.39), heading 1.9 deg, mean speed 0.213 m/s, driven during Interval_Segment_final = [8.902, 13.6] s.
- Segment_prev_1: the path between 1 and 2 m before the fall, from (-2.16, -0.439) to (-1.16, -0.423), heading 0.9 deg, mean speed 0.275 m/s, driven during Interval_Segment_prev_1 = [5.272, 8.902] s.
- Segment_prev_2: the path between 2 and 3 m before the fall, from (-3.159, -0.402) to (-2.16, -0.439), heading -2.1 deg, mean speed 0.31 m/s, driven during Interval_Segment_prev_2 = [2.051, 5.272] s.
- Segment_prev_3: the path between 3 and 3.6 m before the fall, from (-3.7, -0.3) to (-3.159, -0.402), heading -10.7 deg, mean speed 0.305 m/s, driven during Interval_Segment_prev_3 = [0.24, 2.051] s.

### Time intervals (the anchors for a time: `interval`)
- Interval_fall = [11.6, 13.6] s: the last 2 s before the fall.
- Interval_baseline = [1, 11.6] s: free motion before Interval_fall: the reference the peaks are compared with.
- Interval_Phase_1 = [0.24, 1.696] s: the whole of Phase_1 (turn).
- Interval_Phase_2 = [1.696, 13.6] s: the whole of Phase_2 (advance_straight).
- Interval_Segment_final = [8.902, 13.6] s: the drive over the last 1 m of the path before the fall.
- Interval_Segment_prev_1 = [5.272, 8.902] s: the drive over the path between 1 and 2 m before the fall.
- Interval_Segment_prev_2 = [2.051, 5.272] s: the drive over the path between 2 and 3 m before the fall.
- Interval_Segment_prev_3 = [0.24, 2.051] s: the drive over the path between 3 and 3.6 m before the fall.

### Evidence (what the recording shows; "in free motion" is the same quantity over Interval_baseline)
- pitch_rate_peak = 1.999 rad/s at 13.056 s, over Interval_fall; in free motion: 0.006 rad/s. (pitch rate peak: Max |gyroscope x| in the interval (rad/s).)
- vertical_accel_peak = 10.544 m/s2 at 13.12 s, over Interval_fall; in free motion: 0.018 m/s2. (vertical acceleration peak: Max |accelerometer z - g| in the interval (m/s2).)
- longitudinal_accel_peak = 4.877 m/s2 at 12.339 s, over Interval_fall; in free motion: 2.853 m/s2. (longitudinal acceleration peak: Max |accelerometer y| (forward axis) in the interval (m/s2).)
- lateral_accel_peak = 12.027 m/s2 at 13.12 s, over Interval_fall; in free motion: 0.137 m/s2. (lateral acceleration peak: Max |accelerometer x| in the interval (m/s2).)
- yaw_change = 14.83 deg, over Interval_fall; commanded: -8.67 deg. (yaw change: Heading change of the recorded pose over the interval, next to the commanded one: -integral of robot_ref_rot_speed, whose sign is opposite to the executed turn (deg).)
- yaw_rate_peak = 1.053 rad/s at 13.056 s, over Interval_fall; in free motion: 0.268 rad/s. (yaw rate peak: Max |gyroscope z| in the interval (rad/s).)
- setpoint_step = 0.023 m/s, over Interval_fall. (setpoint step: Largest change of the filtered forward setpoint (0.2 s moving median of the zero-order hold) within 0.5 s (m/s).)
- speed_ratio = 0.752, over Interval_fall; travelled over commanded: 0.413 there, 0.55 in free motion. (speed ratio: Path / integral of the setpoint in the interval, divided by the same ratio in Interval_baseline. Relative to the episode itself because of the Webots clock.)
- max_sustained_horizontal_accel = 3.352 m/s2 at 0.305 s, over Interval_Phase_1. (max sustained horizontal acceleration: Max before t_obs of the min of 3 consecutive horizontal acceleration samples (m/s2). Measured in simulated time, so independent of the Webots clock.)
- person_distance_min = 3.473 m, over Interval_fall. (minimum person distance: Minimum robot-person distance in the interval (m).)
- bottle_reacquired = no. (bottle reacquired: Whether the robot->bottle RT edge appears again after t_obs.)

## The mechanisms

The closed list of ways the bottle can fall. Use their ids. Their parameters are kinds only: how big, how strong or how slippery is not chosen, because the memory simulates the whole range it can.

- obstacle_traversed: The robot drives over a low object on the floor (a bump or a cable) that it did not detect; the jolt tips the bottle.
  Anchors: segment. Parameters: shape: bump | cable.
  Precondition: Mean speed on the anchored segment > 0.05 m/s.
  Leaves a trace on: pitch_rate_peak, vertical_accel_peak.
  Simulable: yes. Cost: a bump, 4 simulations (its size is drawn over the whole range); a cable, 1.
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

Shadow is an indoor mobile robot that follows people. While following, it carries a bottle standing, unfastened, on a tray about 0.8 m above the floor.

## Body and Locomotion
- Shadow uses a differential-drive mobile base: 2 drive wheels on a common lateral axis (radius 100 mm, 518 mm apart) and 2 passive caster wheels, one at the front and one at the back.
- It moves forward and backward, turns while moving and rotates in place; it cannot move sideways.
- Each drive wheel has its own hub motor; a single dual-axis SVD48V driver controls both motors.
- The base limits linear speed to 0.9 m/s and rotation speed to 2 rad/s, accelerates at up to 0.5 m/s^2 and brakes at up to 1.0 m/s^2.
- A suspension of steel rods, dampers and springs decouples the wheels from the chassis.
- The robot is at most 625 mm wide.
- A 48 V lithium battery, mounted low in the chassis, powers the robot. It has an external emergency stop button.

## Sensing
- A WT901B IMU measures the accelerations and angular rates of the chassis.
- Two 3D LiDARs detect obstacles: a Helios for forward mapping and a Bpearl for near-hemispherical coverage around the robot.
- A 360-degree RGB camera (Ricoh Theta Z1) detects people.

## Software
- RoboComp interfaces the motors and sensors.
- The CORTEX layer, on a single NVIDIA Jetson Orin, fuses LiDAR and camera data, tracks people and drives the following.

## Limits
- Shadow has no arms, grippers, head or other actuators than its two drive wheels, and no sensors other than those listed here.

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
