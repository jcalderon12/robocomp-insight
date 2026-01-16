import numpy as np
import pybullet as p

class IMU:
    def __init__(self, body_id, dt):
        self.dt = dt
        self.body_id = body_id
        self.prev_linear_vel, _ = p.getBaseVelocity(self.body_id)

        # IMU parameters
        self.bias_acc = np.array([0.05, -0.02, 0.01]) # Bias offset for accelerometer
        self.noise_std = 0.01 # Standar deviation of the white noise w_acc
        self.g_world = np.array([0, 0, 9.81]) # Gravity

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
        self.prev_linear_vel = current_vel # Actualizar para el próximo frame

        # 3. Transformar al sistema de referencia de la IMU (R_W^IMU)
        # getMatrixFromQuaternion devuelve R_IMU^W, usamos la transpuesta para R_W^IMU
        rot_matrix = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
        R_W_IMU = rot_matrix.T

        # 4. Aplicar la fórmula de tu imagen:
        # a_tilde = R_W_IMU * (a_world + g_world) + bias + noise
        noise = np.random.normal(0, self.noise_std, 3)
        
        acc_measured = R_W_IMU @ (acc_linear_world + self.g_world) + self.bias_acc + noise
        
        
        return np.array(acc_measured), np.array(current_angular_vel)