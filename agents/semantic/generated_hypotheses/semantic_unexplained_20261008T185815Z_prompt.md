You explain anomalies of a mobile robot. You propose hypotheses; the robot's memory checks them against the recording and simulates the ones that survive.

## The episode rec_mission_follow_person_08102026_205728

Reference frame: room. Position units: m. Heading convention: deg, room frame, 0 = +x, counter-clockwise.
Times in seconds; origin: first_imu_event.

### Observed discrepancy
- Observation_1: deletion of the robot->bottle RT edge; observation time = 7.161 s.
- Recorded change (removed): subject=PhysicalObject_Bottle; relation=hasLocation; object=Agent_Robot.
- Entities and their roles: Agent_Robot (robot); PhysicalObject_Tray (supporter); PhysicalObject_Bottle (supported, affected entity); Agent_Person (person); PhysicalPlace_Room (room).
- Kind of change, according to the ontology: perceived discontinuity of a carried-object relation.
- Robot pose at the observation: position=[-0.302, -0.301], heading=0.1 deg.
- Support relation in the robot's working memory: PhysicalObject_Tray supporting PhysicalObject_Bottle, over [0, 7.161] s.

### Recorded phases before the observation
- Phase_1 (advance_straight), Interval_Phase_1 = [0.028, 7.161] s: mean speed 0.476 m/s, commanded 0.24-0.59 m/s, heading from 0 to 0.1 deg.

### Recorded path segments (anchors for a place: `segment`)
- Segment_final: recorded path from [-1.302, -0.3] to [-0.302, -0.301], heading -0.1 deg, mean speed 0.434 m/s, during Interval_Segment_final = [4.857, 7.161] s.
- Segment_prev_1: recorded path from [-2.302, -0.3] to [-1.302, -0.3], heading 0 deg, mean speed 0.482 m/s, during Interval_Segment_prev_1 = [2.782, 4.857] s.
- Segment_prev_2: recorded path from [-3.302, -0.3] to [-2.302, -0.3], heading 0 deg, mean speed 0.509 m/s, during Interval_Segment_prev_2 = [0.818, 2.782] s.
- Segment_prev_3: recorded path from [-3.696, -0.3] to [-3.302, -0.3], heading 0 deg, mean speed 0.498 m/s, during Interval_Segment_prev_3 = [0.028, 0.818] s.

### Time intervals (anchors for a time: `interval`)
- Interval_before_observation = [5.161, 7.161] s.
- Interval_baseline = [1, 5.161] s.
- Interval_Phase_1 = [0.028, 7.161] s.
- Interval_Segment_final = [4.857, 7.161] s.
- Interval_Segment_prev_1 = [2.782, 4.857] s.
- Interval_Segment_prev_2 = [0.818, 2.782] s.
- Interval_Segment_prev_3 = [0.028, 0.818] s.

### Evidence (recorded measurements)
- pitch_rate_peak = 0.584 rad/s at 6.908 s, over Interval_before_observation; over Interval_baseline: 0.008 rad/s. (pitch rate peak: Max |gyroscope x| in the interval (rad/s).)
- vertical_accel_peak = 4.914 m/s2 at 6.636 s, over Interval_before_observation; over Interval_baseline: 0.003 m/s2. (vertical acceleration peak: Max |accelerometer z - g| in the interval (m/s2).)
- longitudinal_accel_peak = 3.522 m/s2 at 6.124 s, over Interval_before_observation; over Interval_baseline: 3.522 m/s2. (longitudinal acceleration peak: Max |accelerometer y| (forward axis) in the interval (m/s2).)
- lateral_accel_peak = 0.422 m/s2 at 7.132 s, over Interval_before_observation; over Interval_baseline: 0.001 m/s2. (lateral acceleration peak: Max |accelerometer x| in the interval (m/s2).)
- yaw_change = 0.08 deg, over Interval_before_observation; commanded: 0 deg. (yaw change: Heading change of the recorded pose over the interval, next to the commanded one: -integral of robot_ref_rot_speed, whose sign is opposite to the executed turn (deg).)
- yaw_rate_peak = 0.043 rad/s at 6.028 s, over Interval_before_observation; over Interval_baseline: 0.054 rad/s. (yaw rate peak: Max |gyroscope z| in the interval (rad/s).)
- setpoint_step = 0.04 m/s, over Interval_before_observation. (setpoint step: Largest change of the filtered forward setpoint (0.2 s moving median of the zero-order hold) within 0.5 s (m/s).)
- speed_ratio = 1.048, over Interval_before_observation; travelled over commanded: 0.964 there, 0.92 over Interval_baseline. (speed ratio: Path / integral of the setpoint in the interval, divided by the same ratio in Interval_baseline. Relative to the episode itself because of the Webots clock.)
- max_sustained_horizontal_accel = 3.527 m/s2 at 0.108 s, over Interval_Phase_1. (max sustained horizontal acceleration: Max before t_obs of the min of 3 consecutive horizontal acceleration samples (m/s2). Measured in simulated time, so independent of the Webots clock.)
- person_distance_min = 3.602 m, over Interval_before_observation. (minimum person distance: Minimum robot-person distance in the interval (m).)
- bottle_reacquired = no. (bottle reacquired: Whether the robot->bottle RT edge appears again after t_obs within the available recording.)
- Recording available through 27.051 s.

## The mechanisms

Candidate mechanisms that the ontology declares for this kind of change. Use their ids. Parameters are qualitative kinds only; the memory supplies numerical magnitudes and simulation ranges.

- obstacle_traversed: Agent_Robot could drive over an undetected low object on the floor (a bump or a cable); the disturbance could displace PhysicalObject_Bottle from PhysicalObject_Tray.
  Anchors: segment. Parameters: shape: bump | cable.
  Precondition: Mean speed on the anchored segment > 0.05 m/s.
  Leaves a trace on: pitch_rate_peak, vertical_accel_peak.
  Simulable: yes.
- bottle_push: An external interaction could displace PhysicalObject_Bottle from PhysicalObject_Tray; this need not disturb the carrier.
  Anchors: interval. Parameters: direction: any | forward | backward | left | right.
  Precondition: The recorded support estimate covers the start of the anchored interval.
  Leaves a trace on: nothing the recording measures.
  Simulable: yes.
- robot_push: An external interaction could push Agent_Robot; the disturbance or deviation could displace PhysicalObject_Bottle.
  Anchors: interval. Parameters: direction: any | forward | backward | left | right.
  Precondition: none.
  Leaves a trace on: lateral_accel_peak, speed_ratio, yaw_change.
  Simulable: yes.
- slippery_floor: A slippery floor could make the carrier slip and displace PhysicalObject_Bottle.
  Anchors: none. Parameters: none.
  Precondition: none.
  Leaves a trace on: max_sustained_horizontal_accel, speed_ratio.
  Simulable: yes.
- wheel_failure: A drive wheel of Agent_Robot could stop and remain braked, causing an uncommanded turn that affects PhysicalObject_Bottle.
  Anchors: interval. Parameters: side: left | right | unknown.
  Precondition: Mean speed in the anchored interval > 0.05 m/s.
  Leaves a trace on: speed_ratio, yaw_change, yaw_rate_peak.
  Simulable: yes.
- commanded_speed_change: A recorded command could change the carrier speed sharply and displace PhysicalObject_Bottle.
  Anchors: interval. Parameters: none.
  Precondition: none.
  Leaves a trace on: setpoint_step.
  Simulable: the nominal run, which replays the recorded setpoints, already reproduces it.
- uncommanded_motion: The carrier could brake or jerk without a corresponding recorded command, for example because of a driver or power fault.
  Anchors: interval. Parameters: kind: brake | jerk.
  Precondition: Mean speed in the anchored interval > 0.05 m/s.
  Leaves a trace on: longitudinal_accel_peak, speed_ratio.
  Simulable: not yet: its simulation is experimental.
- bottle_removed_by_person: A person within reach could remove PhysicalObject_Bottle from PhysicalObject_Tray.
  Anchors: interval. Parameters: none.
  Precondition: none.
  Leaves a trace on: person_distance_min.
  Simulable: no. No removal intervention is implemented: this physical hypothesis is checked against the recording but cannot be reproduced by the available simulator.
- perception_failure: Detection, association or tracking of PhysicalObject_Bottle could fail while its physical support remains unchanged.
  Anchors: interval. Parameters: none.
  Precondition: none.
  Leaves a trace on: bottle_reacquired.
  Simulable: no. The available simulator has no detection or tracking model; this hypothesis can only be compared with recorded observations.
- suspension_failure: A suspension fault or vibration of the carrier could disturb the support of PhysicalObject_Bottle.
  Anchors: none. Parameters: none.
  Precondition: none.
  Leaves a trace on: nothing the recording measures.
  Simulable: no. The PyBullet URDF has no suspension.
- new: a mechanism that is not in this list. Describe it in new_mechanism_description; its anchors are optional. It is recorded as unlisted and is not simulated; it does not modify the ontology.

## The robot (its self-model)

# Shadow Robot Self-Model

Shadow is an indoor mobile robot that can follow people and carry unfastened objects on a tray about 0.8 m above the floor.

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

Explain the observed discrepancy Observation_1 described above. Propose candidate mechanisms as hypotheses written in symbols:
- each hypothesis names one mechanism id from the list (or `new`), the anchors that mechanism takes (segment and interval ids from the episode), and its qualitative parameters. Every parameter of the mechanism is required, with one of its listed values;
- never write a number: no coordinates, times, fractions, forces nor probabilities. The memory turns the symbols into numbers;
- read the evidence: a hypothesis should explain what the recording shows;
- order the hypotheses from the most to the least plausible; propose at most 10. Each one is checked against the recording, and every one that survives and can be simulated is simulated;
- propose a mechanism that explains the evidence even if it cannot be simulated.

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
