import numpy as np
from numpy.linalg import inv
import math


import os
import numpy as np
import matplotlib.pyplot as plt
from control import quadrotor_controller_setpoint as pid
import matplotlib.pyplot as plt
import pandas as pd
from PIL import Image

from scipy.spatial.transform import Rotation as R



# Define constants
KC_STATE_DIM = 9  # Dimension of the state vector
KC_STATE_X = 0 #position
KC_STATE_Y = 1
KC_STATE_Z = 2
KC_STATE_PX = 3 #velocities
KC_STATE_PY = 4
KC_STATE_PZ = 5
KC_STATE_D0 = 6 # angular rates
KC_STATE_D1 = 7
KC_STATE_D2 = 8

ROLLPITCH_ZERO_REVERSION = 0.0

GRAVITY_MAGNITUDE = 0 #9.81  # Gravity value

# The bounds on the covariance
MAX_COVARIANCE = 100
MIN_COVARIANCE = 1e-6
EPS = 1e-6


# Supporting and utility functions
def assert_state_not_nan(self):
    """
    Check if any elements of state vector or covariance matrix are NaN.
    """
    assert not np.isnan(self.S).any(), "State vector contains NaN"
    assert not np.isnan(self.q).any(), "Quaternion contains NaN"
    assert not np.isnan(self.P).any(), "Covariance matrix contains NaN"





class kalman_filter():
    def __init__(self):
        """
        Initialize default Kalman filter parameters.
        """
        params = {}
        params['stdDevInitialPosition_xy'] = 1
        params['stdDevInitialPosition_z'] = 1
        params['stdDevInitialVelocity'] = 0.01
        params['stdDevInitialAttitude_rollpitch'] = 0.01
        params['stdDevInitialAttitude_yaw'] = 0.01
        params['procNoiseAcc_xy'] = 0.01
        params['procNoiseAcc_z'] = 0.01
        params['procNoiseVel'] = 0.001
        params['procNoisePos'] = 0.01
        params['procNoiseAtt'] = 0.001
        params['measNoiseBaro'] = 0.01
        params['measNoiseGyro_rollpitch'] = 0.01
        params['measNoiseGyro_yaw'] = 0.01

        params['initialX'] = 0.0
        params['initialY'] = 0.0
        params['initialZ'] = 0.0
        params['initialYaw'] = 0.0

        self.params = params

        #Variables for Plotting
        self.raw_data_vec = []
        self.noisy_data_vec = []
        self.KF_estimate_vec = []
        self.time = []

        # Variable for noise generation
        self.x_noisy_global_last = 0.01
        self.y_noisy_global_last = 0.01
        self.z_noisy_global_last = 0.01
        self.ax_noisy_global_last = 0.01
        self.ay_noisy_global_last = 0.01
        self.az_noisy_global_last = 0.01
        self.v_x_noisy = 0.01
        self.v_y_noisy = 0.01
        self.v_z_noisy = 0.01

        #Initialize KF state
        self.kalman_core_init(self.params)






    def kalman_core_init(self,params):
        """
        Initialize Kalman filter data structure.
        """
        self.S = np.zeros(KC_STATE_DIM)  # State vector
        self.q = np.zeros(4)  # Quaternion
        self.P = np.zeros((KC_STATE_DIM, KC_STATE_DIM))  # Covariance matrix
        self.initialQuaternion = np.zeros(4)  # Initial quaternion
        self.R = np.eye(3)  # Rotation matrix
        self.baroReferenceHeight = 0.0  # Barometer reference height

        # Initialize state
        self.S[KC_STATE_X] = params['initialX']
        self.S[KC_STATE_Y] = params['initialY']
        self.S[KC_STATE_Z] = params['initialZ']

        # Initialize quaternion
        self.initialQuaternion[0] = np.cos(params['initialYaw'] / 2)
        self.initialQuaternion[3] = np.sin(params['initialYaw'] / 2)
        self.q = self.initialQuaternion.copy()

        # Initialize covariance matrix
        std_dev_pos_xy =    params['stdDevInitialPosition_xy']
        std_dev_pos_z =     params['stdDevInitialPosition_z']
        std_dev_vel =       params['stdDevInitialVelocity']
        std_dev_att_rp =    params['stdDevInitialAttitude_rollpitch']
        std_dev_att_yaw =   params['stdDevInitialAttitude_yaw']

        self.P[KC_STATE_X][KC_STATE_X] = std_dev_pos_xy ** 2
        self.P[KC_STATE_Y][KC_STATE_Y] = std_dev_pos_xy ** 2
        self.P[KC_STATE_Z][KC_STATE_Z] = std_dev_pos_z ** 2
        self.P[KC_STATE_PX][KC_STATE_PX] = std_dev_vel ** 2
        self.P[KC_STATE_PY][KC_STATE_PY] = std_dev_vel ** 2
        self.P[KC_STATE_PZ][KC_STATE_PZ] = std_dev_vel ** 2
        self.P[KC_STATE_D0][KC_STATE_D0] = std_dev_att_rp ** 2
        self.P[KC_STATE_D1][KC_STATE_D1] = std_dev_att_rp ** 2
        self.P[KC_STATE_D2][KC_STATE_D2] = std_dev_att_yaw ** 2

        # self.P = np.ones(KC_STATE_DIM,dtype=float)





    def kalman_core_scalar_update(self, Hm, error, std_meas_noise):
        """
        Update the Kalman filter state based on a scalar measurement.
        """
        # The Kalman gain as a column vector
        K = np.zeros(KC_STATE_DIM)

        # Temporary matrices for the covariance updates
        tmpNN1m = np.zeros((KC_STATE_DIM, KC_STATE_DIM))
        tmpNN2m = np.zeros((KC_STATE_DIM, KC_STATE_DIM))
        tmpNN3m = np.zeros((KC_STATE_DIM, KC_STATE_DIM))
        PHTm = np.zeros((KC_STATE_DIM, 1))

        assert Hm.shape == (1, KC_STATE_DIM)

        # ====== INNOVATION COVARIANCE ======
        print(self.P.shape)
        PHTm = np.dot(self.P, Hm.T)  # PH'
        print(PHTm.shape)
        R = std_meas_noise ** 2
        HPHR = R + np.dot(Hm, PHTm)  # HPH' + R
        assert not np.isnan(HPHR)

        # ====== MEASUREMENT UPDATE ======
        # Calculate the Kalman gain and perform the state update
        for i in range(KC_STATE_DIM):
            K[i] = PHTm[i,0] / HPHR  # Kalman gain = (PH' (HPH' + R )^-1)
            self.S[i] += K[i] * error  # State update
        print(K)

        # ====== COVARIANCE UPDATE ======
        Km = K.reshape(-1, 1)
        tmpNN1m = np.dot(Km, Hm)
        np.fill_diagonal(tmpNN1m, np.diagonal(tmpNN1m) - 1) # KH - I
        tmpNN2m = tmpNN1m.T # (KH - I)'
        tmpNN3m = np.dot(tmpNN1m, self.P) # (KH - I)*P
        tmpNN3m = np.dot(tmpNN3m, tmpNN2m) # (KH - I)*P*(KH - I)'
        self.P = tmpNN3m
        print(self.P)
        # Add the measurement variance and ensure boundedness and symmetry
        for i in range(KC_STATE_DIM):
            for j in range(i, KC_STATE_DIM):
                v = K[i] * R * K[j]
                p = 0.5 * self.P[i][j] + 0.5 * self.P[j][i] + v  # Add measurement noise
                if np.isnan(p) or p > MAX_COVARIANCE:
                    self.P[i][j] = self.P[j][i] = MAX_COVARIANCE
                elif i == j and p < MIN_COVARIANCE:
                    self.P[i][j] = self.P[j][i] = MIN_COVARIANCE
                else:
                    self.P[i][j] = self.P[j][i] = p
        print(self.P)








    def kalmanCorePredict(self, acc, gyro, dt, quad_is_flying):
        EPS = np.finfo(float).eps

        acc = acc @ self.R.T


        # Linearized update matrix
        A = np.eye(KC_STATE_DIM)

        # Dynamics linearization
        A[0:3, 3:6] = np.array(self.R) * dt
        A[0:3, 6:9] = np.array([[self.S[KC_STATE_PY] * self.R[0][2] - self.S[KC_STATE_PZ] * self.R[0][1],
                                self.S[KC_STATE_PY] * self.R[1][2] - self.S[KC_STATE_PZ] * self.R[1][1],
                                self.S[KC_STATE_PY] * self.R[2][2] - self.S[KC_STATE_PZ] * self.R[2][1]],
                                [-self.S[KC_STATE_PX] * self.R[0][2] + self.S[KC_STATE_PZ] * self.R[0][0],
                                -self.S[KC_STATE_PX] * self.R[1][2] + self.S[KC_STATE_PZ] * self.R[1][0],
                                -self.S[KC_STATE_PX] * self.R[2][2] + self.S[KC_STATE_PZ] * self.R[2][0]],
                                [self.S[KC_STATE_PX] * self.R[0][1] - self.S[KC_STATE_PY] * self.R[0][0],
                                self.S[KC_STATE_PX] * self.R[1][1] - self.S[KC_STATE_PY] * self.R[1][0],
                                self.S[KC_STATE_PX] * self.R[2][1] - self.S[KC_STATE_PY] * self.R[2][0]]]) * dt
        # body-frame velocity from body-frame velocity
        A[3][3] = 1
        A[4][3] = -gyro[2] * dt
        A[5][3] = gyro[1] * dt

        A[3][4] = gyro[2] * dt
        A[4][4] = 1
        A[5][4] = -gyro[0] * dt

        A[3][5] = -gyro[1] * dt
        A[4][5] = gyro[0] * dt
        A[5][5] = 1
        # body-frame velocity from attitude error
        A[3][6] = 0
        A[4][6] = -GRAVITY_MAGNITUDE * self.R[2][2] * dt
        A[5][6] = GRAVITY_MAGNITUDE * self.R[2][1] * dt

        A[3][7] = GRAVITY_MAGNITUDE * self.R[2][2] * dt
        A[4][7] = 0
        A[5][7] = -GRAVITY_MAGNITUDE * self.R[2][0] * dt

        A[3][8] = -GRAVITY_MAGNITUDE * self.R[2][1] * dt
        A[4][8] = GRAVITY_MAGNITUDE * self.R[2][0] * dt
        A[5][8] = 0
        # attitude error from attitude error
        d0 = gyro[0] * dt / 2
        d1 = gyro[1] * dt / 2
        d2 = gyro[2] * dt / 2
        A[6][6] = 1 - d1 * d1 / 2 - d2 * d2 / 2
        A[6][7] = d2 + d0 * d1 / 2
        A[6][8] = -d1 + d0 * d2 / 2
        A[7][6] = -d2 + d0 * d1 / 2
        A[7][7] = 1 - d0 * d0 / 2 - d2 * d2 / 2
        A[7][8] = d0 + d1 * d2 / 2
        A[8][6] = d1 + d0 * d2 / 2
        A[8][7] = -d0 + d1 * d2 / 2
        A[8][8] = 1 - d0 * d0 / 2 - d1 * d1 / 2


        # ====== Covariance update ====== #
        tmpNN1m = np.dot(A, np.array(self.P)) # A*P
        tmpNN2m = np.transpose(A) # A'
        self.P = np.dot(tmpNN1m, tmpNN2m) # APA'
        # Process noise is added after the return from the prediction step


        # ====== PREDICTION STEP ======
        # The prediction depends on whether we're on the ground, or in flight.
        # When flying, the accelerometer directly measures thrust (hence is useless to estimate body angle while flying)

        dx, dy, dz = 0.0, 0.0, 0.0
        tmp_sp_x, tmp_sp_y, tmp_sp_z = 0.0, 0.0, 0.0
        z_acc = 0.0

        if quad_is_flying:  # Only acceleration in the z direction
            # Use accelerometer and not commanded thrust, as this has proper physical units
            z_acc = acc[2]

            # Position updates in the body frame (will be rotated to inertial frame)
            dx = self.S[KC_STATE_PX] * dt
            dy = self.S[KC_STATE_PY] * dt
            dz = self.S[KC_STATE_PZ] * dt + z_acc * dt ** 2 / 2.0  # Thrust can only be produced in the body's Z direction

            # Position update
            self.S[KC_STATE_X] += self.R[0][0] * dx + self.R[0][1] * dy + self.R[0][2] * dz
            self.S[KC_STATE_Y] += self.R[1][0] * dx + self.R[1][1] * dy + self.R[1][2] * dz
            self.S[KC_STATE_Z] += self.R[2][0] * dx + self.R[2][1] * dy + self.R[2][2] * dz - GRAVITY_MAGNITUDE * dt ** 2 / 2.0

            # Keep previous time step's state for the update
            tmp_sp_x = self.S[KC_STATE_PX]
            tmp_sp_y = self.S[KC_STATE_PY]
            tmp_sp_z = self.S[KC_STATE_PZ]

            # Body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
            self.S[KC_STATE_PX] += dt * (gyro[2] * tmp_sp_y - gyro[1] * tmp_sp_z - GRAVITY_MAGNITUDE * self.R[2][0])
            self.S[KC_STATE_PY] += dt * (-gyro[2] * tmp_sp_x + gyro[0] * tmp_sp_z - GRAVITY_MAGNITUDE * self.R[2][1])
            self.S[KC_STATE_PZ] += dt * (z_acc + gyro[1] * tmp_sp_x - gyro[0] * tmp_sp_y - GRAVITY_MAGNITUDE * self.R[2][2])
        else:  # Acceleration can be in any direction, as measured by the accelerometer. This occurs, eg. in freefall or while being carried.
            # Position updates in the body frame (will be rotated to inertial frame)
            dx = self.S[KC_STATE_PX] * dt + acc[0] * dt ** 2 / 2.0
            dy = self.S[KC_STATE_PY] * dt + acc[1] * dt ** 2 / 2.0
            dz = self.S[KC_STATE_PZ] * dt + acc[2] * dt ** 2 / 2.0  # Thrust can only be produced in the body's Z direction

            # Position update
            self.S[KC_STATE_X] += self.R[0][0] * dx + self.R[0][1] * dy + self.R[0][2] * dz
            self.S[KC_STATE_Y] += self.R[1][0] * dx + self.R[1][1] * dy + self.R[1][2] * dz
            self.S[KC_STATE_Z] += self.R[2][0] * dx + self.R[2][1] * dy + self.R[2][2] * dz - GRAVITY_MAGNITUDE * dt ** 2 / 2.0

            # Keep previous time step's state for the update
            tmp_sp_x = self.S[KC_STATE_PX]
            tmp_sp_y = self.S[KC_STATE_PY]
            tmp_sp_z = self.S[KC_STATE_PZ]

            # Body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
            self.S[KC_STATE_PX] += dt * (acc[0] + gyro[2] * tmp_sp_y - gyro[1] * tmp_sp_z - GRAVITY_MAGNITUDE * self.R[2][0])
            self.S[KC_STATE_PY] += dt * (acc[1] - gyro[2] * tmp_sp_x + gyro[0] * tmp_sp_z - GRAVITY_MAGNITUDE * self.R[2][1])
            self.S[KC_STATE_PZ] += dt * (acc[2] + gyro[1] * tmp_sp_x - gyro[0] * tmp_sp_y - GRAVITY_MAGNITUDE * self.R[2][2])

        # Attitude update (rotate by gyroscope), we do this in quaternions
        # This is the gyroscope angular velocity integrated over the sample period
        dtwx = dt * gyro[0]
        dtwy = dt * gyro[1]
        dtwz = dt * gyro[2]

        # Compute the quaternion values in [w,x,y,z] order
        angle = np.sqrt(dtwx ** 2 + dtwy ** 2 + dtwz ** 2) + EPS
        ca = np.cos(angle / 2.0)
        sa = np.sin(angle / 2.0)
        dq = [ca, sa * dtwx / angle, sa * dtwy / angle, sa * dtwz / angle]

        # Rotate the quad's attitude by the delta quaternion vector computed above
        tmpq0 = dq[0] * self.q[0] - dq[1] * self.q[1] - dq[2] * self.q[2] - dq[3] * self.q[3]
        tmpq1 = dq[1] * self.q[0] + dq[0] * self.q[1] + dq[3] * self.q[2] - dq[2] * self.q[3]
        tmpq2 = dq[2] * self.q[0] - dq[3] * self.q[1] + dq[0] * self.q[2] + dq[1] * self.q[3]
        tmpq3 = dq[3] * self.q[0] + dq[2] * self.q[1] - dq[1] * self.q[2] + dq[0] * self.q[3]

        if not quad_is_flying:
            keep = 1.0 - ROLLPITCH_ZERO_REVERSION

            tmpq0 = keep * tmpq0 + ROLLPITCH_ZERO_REVERSION * self.initialQuaternion[0]
            tmpq1 = keep * tmpq1 + ROLLPITCH_ZERO_REVERSION * self.initialQuaternion[1]
            tmpq2 = keep * tmpq2 + ROLLPITCH_ZERO_REVERSION * self.initialQuaternion[2]
            tmpq3 = keep * tmpq3 + ROLLPITCH_ZERO_REVERSION * self.initialQuaternion[3]

        # Normalize and store the result
        norm = np.sqrt(tmpq0 ** 2 + tmpq1 ** 2 + tmpq2 ** 2 + tmpq3 ** 2) + EPS
        self.q[0] = tmpq0 / norm
        self.q[1] = tmpq1 / norm
        self.q[2] = tmpq2 / norm
        self.q[3] = tmpq3 / norm
        assert_state_not_nan(self)








    def kalman_core_add_process_noise(self, params, dt):
        if dt > 0:
            # Add process noise on position
            pos_noise = (params['procNoiseAcc_xy'] * dt ** 2 + params['procNoiseVel'] * dt + params['procNoisePos']) ** 2
            self.P[KC_STATE_X][KC_STATE_X] += pos_noise
            self.P[KC_STATE_Y][KC_STATE_Y] += pos_noise
            self.P[KC_STATE_Z][KC_STATE_Z] += pos_noise

            # Add process noise on velocity
            vel_noise = (params['procNoiseAcc_xy'] * dt + params['procNoiseVel']) ** 2
            self.P[KC_STATE_PX][KC_STATE_PX] += vel_noise
            self.P[KC_STATE_PY][KC_STATE_PY] += vel_noise
            self.P[KC_STATE_PZ][KC_STATE_PZ] += vel_noise

            # Add process noise on angular velocity
            self.P[KC_STATE_D0][KC_STATE_D0] += (params['measNoiseGyro_rollpitch'] * dt + params['procNoiseAtt']) ** 2
            self.P[KC_STATE_D1][KC_STATE_D1] += (params['measNoiseGyro_rollpitch'] * dt + params['procNoiseAtt']) ** 2
            self.P[KC_STATE_D2][KC_STATE_D2] += (params['measNoiseGyro_yaw'] * dt + params['procNoiseAtt']) ** 2

        for i in range(KC_STATE_DIM):
            for j in range(i, KC_STATE_DIM):
                p = 0.5 * self.P[i][j] + 0.5 * self.P[j][i]
                if np.isnan(p) or p > MAX_COVARIANCE:
                    self.P[i][j] = self.P[j][i] = MAX_COVARIANCE
                elif i == j and p < MIN_COVARIANCE:
                    self.P[i][j] = self.P[j][i] = MIN_COVARIANCE
                else:
                    self.P[i][j] = self.P[j][i] = p

        assert_state_not_nan(self)






    def kalman_core_finalize(self):
        # Matrix to rotate the attitude covariances once updated
        A = np.zeros((KC_STATE_DIM, KC_STATE_DIM))

        # Incorporate the attitude error (Kalman filter state) with the attitude
        v0 = self.S[KC_STATE_D0]
        v1 = self.S[KC_STATE_D1]
        v2 = self.S[KC_STATE_D2]

        # # Move attitude error into attitude if any of the angle errors are large enough
        # if (abs(v0) > 0.1e-3 or abs(v1) > 0.1e-3 or abs(v2) > 0.1e-3) and (abs(v0) < 10 and abs(v1) < 10 and abs(v2) < 10):
        #     angle = np.sqrt(v0 * v0 + v1 * v1 + v2 * v2)
        #     ca = np.cos(angle / 2.0)
        #     sa = np.sin(angle / 2.0)
        #     dq = np.array([ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle])

        #     # Rotate the quad's attitude by the delta quaternion vector computed above
        #     tmpq0 = dq[0] * self.q[0] - dq[1] * self.q[1] - dq[2] * self.q[2] - dq[3] * self.q[3]
        #     tmpq1 = dq[1] * self.q[0] + dq[0] * self.q[1] + dq[3] * self.q[2] - dq[2] * self.q[3]
        #     tmpq2 = dq[2] * self.q[0] - dq[3] * self.q[1] + dq[0] * self.q[2] + dq[1] * self.q[3]
        #     tmpq3 = dq[3] * self.q[0] + dq[2] * self.q[1] - dq[1] * self.q[2] + dq[0] * self.q[3]

        #     # Normalize and store the result
        #     norm = np.sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3)
        #     self.q[0] = tmpq0 / norm
        #     self.q[1] = tmpq1 / norm
        #     self.q[2] = tmpq2 / norm
        #     self.q[3] = tmpq3 / norm

        #     self.q /= np.norm(self.q)
        #     # Rotate the covariance
        #     d0 = v0 / 2
        #     d1 = v1 / 2
        #     d2 = v2 / 2

        #     A[KC_STATE_X][KC_STATE_X] = 1
        #     A[KC_STATE_Y][KC_STATE_Y] = 1
        #     A[KC_STATE_Z][KC_STATE_Z] = 1

        #     A[KC_STATE_PX][KC_STATE_PX] = 1
        #     A[KC_STATE_PY][KC_STATE_PY] = 1
        #     A[KC_STATE_PZ][KC_STATE_PZ] = 1

        #     A[KC_STATE_D0][KC_STATE_D0] = 1 - d1 * d1 / 2 - d2 * d2 / 2
        #     A[KC_STATE_D0][KC_STATE_D1] = d2 + d0 * d1 / 2
        #     A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0 * d2 / 2

        #     A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0 * d1 / 2
        #     A[KC_STATE_D1][KC_STATE_D1] = 1 - d0 * d0 / 2 - d2 * d2 / 2
        #     A[KC_STATE_D1][KC_STATE_D2] = d0 + d1 * d2 / 2

        #     A[KC_STATE_D2][KC_STATE_D0] = d1 + d0 * d2 / 2
        #     A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1 * d2 / 2
        #     A[KC_STATE_D2][KC_STATE_D2] = 1 - d0 * d0 / 2 - d1 * d1 / 2

        #     Am = np.transpose(A)
        #     tmpNN2m = np.dot(Am, self.P)
        #     self.P = np.dot(tmpNN2m, Am)

        # # Convert the new attitude to a rotation matrix
        # self.R[0][0] = self.q[0] * self.q[0] + self.q[1] * self.q[1] - self.q[2] * self.q[2] - self.q[3] * self.q[3]
        # self.R[0][1] = 2 * self.q[1] * self.q[2] - 2 * self.q[0] * self.q[3]
        # self.R[0][2] = 2 * self.q[1] * self.q[3] + 2 * self.q[0] * self.q[2]

        # self.R[1][0] = 2 * self.q[1] * self.q[2] + 2 * self.q[0] * self.q[3]
        # self.R[1][1] = self.q[0] * self.q[0] - self.q[1] * self.q[1] + self.q[2] * self.q[2] - self.q[3] * self.q[3]
        # self.R[1][2] = 2 * self.q[2] * self.q[3] - 2 * self.q[0] * self.q[1]

        # self.R[2][0] = 2 * self.q[1] * self.q[3] - 2 * self.q[0] * self.q[2]
        # self.R[2][1] = 2 * self.q[2] * self.q[3] + 2 * self.q[0] * self.q[1]
        # self.R[2][2] = self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]

        q0, q1, q2, q3 = self.q
    
        # Compute rotation matrix
        R_temp = np.array([
            [1 - 2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q3*q0, 2*q1*q3 + 2*q2*q0],
            [2*q1*q2 + 2*q3*q0, 1 - 2*q1**2 - 2*q3**2, 2*q2*q3 - 2*q1*q0],
            [2*q1*q3 - 2*q2*q0, 2*q2*q3 + 2*q1*q0, 1 - 2*q1**2 - 2*q2**2]
            ])
        # r = R.from_euler('xyz', [data['roll'], data['pitch'], data['yaw']])
        # R_T = r.as_matrix()
        self.R = R_temp
        # Reset the attitude error
        self.S[KC_STATE_D0] = 0
        self.S[KC_STATE_D1] = 0
        self.S[KC_STATE_D2] = 0

        # Enforce symmetry of the covariance matrix and ensure the values stay bounded
        for i in range(KC_STATE_DIM):
            for j in range(i, KC_STATE_DIM):
                p = 0.5 * self.P[i][j] + 0.5 * self.P[j][i]
                if np.isnan(p) or p > MAX_COVARIANCE:
                    self.P[i][j] = self.P[j][i] = MAX_COVARIANCE
                elif i == j and p < MIN_COVARIANCE:
                    self.P[i][j] = self.P[j][i] = MIN_COVARIANCE
                else:
                    self.P[i][j] = self.P[j][i] = p

        assert_state_not_nan(self)




    # def kalman_core_externalize_state(self, state, acc, tick):
    #     # Position state is already in the world frame
    #     state.position = {
    #         'timestamp': tick,
    #         'x': self.S[KC_STATE_X],
    #         'y': self.S[KC_STATE_Y],
    #         'z': self.S[KC_STATE_Z]
    #     }

    #     # Velocity is in the body frame and needs to be rotated to the world frame
    #     state.velocity = {
    #         'timestamp': tick,
    #         'x': self.R[0][0]*self.S[KC_STATE_PX] + self.R[0][1]*self.S[KC_STATE_PY] + self.R[0][2]*self.S[KC_STATE_PZ],
    #         'y': self.R[1][0]*self.S[KC_STATE_PX] + self.R[1][1]*self.S[KC_STATE_PY] + self.R[1][2]*self.S[KC_STATE_PZ],
    #         'z': self.R[2][0]*self.S[KC_STATE_PX] + self.R[2][1]*self.S[KC_STATE_PY] + self.R[2][2]*self.S[KC_STATE_PZ]
    #     }

    #     # Accelerometer measurements are in the body frame and need to be rotated to the world frame
    #     # Furthermore, the legacy code requires acc.z to be acceleration without gravity
    #     # Finally, note that these accelerations are in Gs, and not in m/s^2, hence - 1 for removing gravity
    #     state.acc = {
    #         'timestamp': tick,
    #         'x': self.R[0][0]*acc.x + self.R[0][1]*acc.y + self.R[0][2]*acc.z,
    #         'y': self.R[1][0]*acc.x + self.R[1][1]*acc.y + self.R[1][2]*acc.z,
    #         'z': self.R[2][0]*acc.x + self.R[2][1]*acc.y + self.R[2][2]*acc.z - 1
    #     }

    #     # Convert the new attitude into Euler YPR
    #     yaw = math.atan2(2*(self.q[1]*self.q[2]+self.q[0]*self.q[3]) , self.q[0]*self.q[0] + self.q[1]*self.q[1] - self.q[2]*self.q[2] - self.q[3]*self.q[3])
    #     pitch = math.asin(-2*(self.q[1]*self.q[3] - self.q[0]*self.q[2]))
    #     roll = math.atan2(2*(self.q[2]*self.q[3]+self.q[0]*self.q[1]) , self.q[0]*self.q[0] - self.q[1]*self.q[1] - self.q[2]*self.q[2] + self.q[3]*self.q[3])

    #     # Save attitude, adjusted for the legacy CF2 body coordinate system
    #     RAD_TO_DEG = 180/np.pi
    #     state.attitude = {
    #         'timestamp': tick,
    #         'roll': roll * RAD_TO_DEG,
    #         'pitch': -pitch * RAD_TO_DEG,
    #         'yaw': yaw * RAD_TO_DEG
    #     }

    #     # Save quaternion, hopefully one day this could be used in a better controller
    #     # Note that this is not adjusted for the legacy coordinate system
    #     state.attitude_quaternion = {
    #         'timestamp': tick,
    #         'w': self.q[0],
    #         'x': self.q[1],
    #         'y': self.q[2],
    #         'z': self.q[3]
    #     }

    #     assert_state_not_nan(self)

    def kalman_core_update_with_position(self, params, pos):
        """
        Update the Kalman filter with position measurements.

        Args:
        - this: Instance of the Kalman filter data structure.
        - pos: Position measurement containing x, y, and z coordinates along with standard deviation.

        This function updates the Kalman filter state based on position measurements.
        """
        # Iterate over each dimension (x, y, z) for direct measurement
        for i in range(3):
            # Initialize a measurement matrix 'h' with zeros
            h = [[0] * KC_STATE_DIM]
            h = np.array(h)
            
            # Set the corresponding element in 'h' to 1, indicating direct measurement of state
            h[0][KC_STATE_X + i] = 1

            # Calculate the residual (difference between measured position and predicted position)
            residual = pos[i] - self.S[KC_STATE_X + i]

            # Update the Kalman filter state for the current dimension with scalar update
            # Scalar update uses only one row of the measurement matrix 'h'
            self.kalman_core_scalar_update( h, residual, params['procNoisePos'])


    def kalman_core_externalize_state(self, KF_state_outputs, acc):
        """
        Externalize the state of the Kalman filter to a specified state structure.

        Args:
        - self: Instance of the Kalman filter data structure.
        - KF_state_outputs: Dictionary to hold the externalized state.
        - acc: Accelerometer measurements.
        - tick: Timestamp for the state.

        This function updates the external state structure with the Kalman filter's current state.
        """

        # Update position state (already in the global frame)
        KF_state_outputs['x_global'] = self.S[KC_STATE_X]
        KF_state_outputs['y_global'] = self.S[KC_STATE_Y]
        KF_state_outputs['z_global'] = self.S[KC_STATE_Z]

        # # Update velocity state (in the global frame)
        # KF_state_outputs['v_x'] = self.R[0][0]*self.S[KC_STATE_PX] + self.R[0][1]*self.S[KC_STATE_PY] + self.R[0][2]*self.S[KC_STATE_PZ]
        # KF_state_outputs['v_y'] = self.R[1][0]*self.S[KC_STATE_PX] + self.R[1][1]*self.S[KC_STATE_PY] + self.R[1][2]*self.S[KC_STATE_PZ]
        # KF_state_outputs['v_z'] = self.R[2][0]*self.S[KC_STATE_PX] + self.R[2][1]*self.S[KC_STATE_PY] + self.R[2][2]*self.S[KC_STATE_PZ]

        # yaw = math.atan2(2*(self.q[0]*self.q[3] + self.q[1]*self.q[2]), 1 - 2*(self.q[2]*self.q[2] + self.q[3]*self.q[3]))
        # pitch = math.asin(2*(self.q[0]*self.q[2] - self.q[3]*self.q[1]))
        # roll = math.atan2(2*(self.q[0]*self.q[1] + self.q[2]*self.q[3]), 1 - 2*(self.q[1]*self.q[1] + self.q[2]*self.q[2]))

        # # Calculate forward, left, and downward velocity components
        # KF_state_outputs['v_forward'] = KF_state_outputs['v_x'] * np.cos(yaw) + KF_state_outputs['v_y'] * np.sin(yaw)
        # KF_state_outputs['v_left'] = -KF_state_outputs['v_x'] * np.sin(yaw) + KF_state_outputs['v_y'] * np.cos(yaw)
        # KF_state_outputs['v_down'] = KF_state_outputs['v_z']

        # # Update accelerometer measurements (in the global frame)
        # KF_state_outputs['ax_global'] = self.R[0][0]*acc[0] + self.R[0][1]*acc[1] + self.R[0][2]*acc[2]
        # KF_state_outputs['ay_global'] = self.R[1][0]*acc[0] + self.R[1][1]*acc[1] + self.R[1][2]*acc[2]
        # KF_state_outputs['az_global'] = self.R[2][0]*acc[0] + self.R[2][1]*acc[1] + self.R[2][2]*acc[2]

        # # Save attitude, adjusted for the legacy CF2 body coordinate system
        # # RAD_TO_DEG = 180/np.pi
        # KF_state_outputs['roll'] = roll
        # KF_state_outputs['pitch'] = -pitch 
        # KF_state_outputs['yaw'] = yaw
        return KF_state_outputs
