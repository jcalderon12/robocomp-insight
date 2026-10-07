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
