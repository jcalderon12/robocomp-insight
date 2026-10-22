"""Simulated IMU, as the Accelerometer and Gyro of the Webots Shadow report it.

Both devices sit at the robot origin (the `imu` link of the URDF) and report in the robot frame
(x right, y forward, z up), as the recording does: at rest the accelerometer reads +9.81 on z, and
the base accelerating forward reads positive on y.

* Accelerometer: the specific force R^T (a - g), with a the mean acceleration of the device point
  over the last `window_s` seconds of physics, i.e. its velocity change over that span. A device
  read once per physics step of the recorded world reports exactly that, with the span of its step.
* Gyroscope: the angular velocity of the robot in its own frame, R^T w.

Neither signal is filtered: the recorded ones are not.
"""
from collections import deque

import numpy as np
import pybullet as p


class IMU:
    def __init__(self, body_id, dt, link_index=-1, window_s=None, gravity=-9.81):
        """`dt`: physics step (s). `link_index`: link the devices are mounted on (-1, the base).
        `window_s`: span of physics the accelerometer averages over (one step by default)."""
        self.body_id = body_id
        self.link_index = link_index
        self.dt = dt
        self.window_steps = max(1, int(round((window_s or dt) / dt)))
        self.gravity_world = np.array([0.0, 0.0, gravity])
        self.reset()

    def _state(self):
        """(rotation world <- device, linear velocity of the device point, angular velocity), in
        the world frame."""
        if self.link_index < 0:
            _, quaternion = p.getBasePositionAndOrientation(self.body_id)
            linear, angular = p.getBaseVelocity(self.body_id)
        else:
            state = p.getLinkState(self.body_id, self.link_index, computeLinkVelocity=1,
                                   computeForwardKinematics=1)
            # The device link has its centre of mass at its origin, so the velocity PyBullet gives
            # for the link (of its centre of mass) is the velocity of the device point.
            quaternion, linear, angular = state[5], state[6], state[7]
        rotation = np.array(p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
        return rotation, np.array(linear), np.array(angular)

    def reset(self):
        """Start over from the current state, as if the device had been at rest at this velocity:
        call it when the bodies are restored, or the first sample differences against the end of
        the previous repetition."""
        _, linear, _ = self._state()
        self._velocities = deque([linear] * (self.window_steps + 1), maxlen=self.window_steps + 1)

    def get_measurement(self):
        """(accelerometer, gyroscope) in the device frame, after the physics step just taken.
        Call it once per step: the accelerometer differences the velocities of the last steps."""
        rotation, linear, angular = self._state()
        self._velocities.append(linear)
        acceleration = (self._velocities[-1] - self._velocities[0]) / (self.window_steps * self.dt)
        world_to_device = rotation.T
        accelerometer = world_to_device @ (acceleration - self.gravity_world)
        gyroscope = world_to_device @ angular
        return accelerometer, gyroscope
