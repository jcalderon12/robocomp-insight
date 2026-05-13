import numpy as np
import pybullet as p

class IMU:
    def __init__(self, body_id, dt):
        self.dt = dt
        self.body_id = body_id
        self.prev_linear_vel, _ = p.getBaseVelocity(self.body_id)

        # IMU parameters
        self.bias_acc = np.array([0.0, 0.0, 0.0]) # Bias offset for accelerometer (set to 0)
        self.noise_std = 0.0 # Standar deviation of the white noise w_acc (disabled)
        self.g_world = np.array([0, 0, 9.81]) # Gravity
        
        # Low-pass filter for acceleration (reduces numerical differentiation noise)
        self.alpha = 0.2  # Filter coefficient (0.0 = max filtering, 1.0 = no filtering)
        self.prev_acc_filtered = np.array([0.0, 0.0, 0.0])  # Previous filtered acceleration
        
        # Low-pass filter for gyroscope (angular velocity)
        self.prev_angular_vel_filtered = np.array([0.0, 0.0, 0.0])  # Previous filtered angular velocity

    def get_measurement(self):
        """Simulate IMU accelerometer measurement.
            Accelerometer measurement based on the IMU model described in: ã_imu = R * (a + g) + b + w
            R: Rotation matrix from world to IMU frame
            a: Linear acceleration in world frame
            g: Gravity vector in world frame
            b: Bias offset
            w: White noise

        Returns:
            acc_measured: Simulated accelerometer measurement (numpy array of shape (3,))
            quat: Current orientation of the IMU in quaternion format (x, y, z, w)

        """
        # 1. Obtener estado actual de PyBullet
        pos, quat = p.getBasePositionAndOrientation(self.body_id)
        current_vel, current_angular_vel = p.getBaseVelocity(self.body_id)
        
        # 2. Calcular la derivada: a = (v_t - v_t-1) / dt
        acc_linear_world = (np.array(current_vel) - np.array(self.prev_linear_vel)) / self.dt
        
        # Apply low-pass filter to reduce numerical differentiation noise
        acc_linear_world = self.alpha * acc_linear_world + (1.0 - self.alpha) * self.prev_acc_filtered
        self.prev_acc_filtered = acc_linear_world
        
        self.prev_linear_vel = current_vel # Actualizar para el próximo frame

        # 3. Transformar al sistema de referencia de la IMU (R_W^IMU)
        # getMatrixFromQuaternion devuelve R_IMU^W, usamos la transpuesta para R_W^IMU
        rot_matrix = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
        R_W_IMU = rot_matrix.T

        # 4. Aplicar la fórmula sin ruido ni bias (ya desactivados)
        acc_measured = R_W_IMU @ (acc_linear_world) + self.g_world
        
        # Apply low-pass filter to gyroscope (angular velocity)
        angular_vel = np.array(current_angular_vel)
        angular_vel_filtered = self.alpha * angular_vel + (1.0 - self.alpha) * self.prev_angular_vel_filtered
        self.prev_angular_vel_filtered = angular_vel_filtered
        
        return np.array(acc_measured), angular_vel_filtered