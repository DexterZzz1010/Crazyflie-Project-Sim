import numpy as np
import math


class pid_pos_controller():
    def __init__(self):
        self.past_x_error = 0.0
        self.past_y_error = 0.0
        self.past_vx_error = 0.0
        self.past_vy_error = 0.0
        self.past_alt_error = 0.0
        self.past_pitch_error = 0.0
        self.past_roll_error = 0.0
        self.past_yaw_error = 0.0
        self.altitude_integrator = 0.0
        
        self.past_pitch_rate_error = 0.0
        self.past_roll_rate_error = 0.0
        self.past_yaw_rate_error = 0.0
        
        self.past_roll_rate_error = 0.0
        self.past_pitch_rate_error = 0.0
        self.past_yaw_rate_error = 0.0
        
        self.pitch_rate_integrator = 0.0
        self.roll_rate_integrator = 0.0
        self.yaw_rate_integrator = 0.0
         
        self.last_time = 0.0

    def pid(self, dt, pos_desired, angle_desired, 
                                        act_roll, act_pitch, act_yaw, act_roll_rate, act_pitch_rate, act_yaw_rate,
                                        act_pos, actual_vx, actual_vy):
        # Velocity PID control (converted from Crazyflie c code)
        gains = {"kp_att_y": 0.8, "kd_att_y": 0.5, "kp_att_rp": 0.5, "kd_att_rp": 0.1,
                 "kp_vel_xy": 2, "kd_vel_xy": 0.5, "kp_z": 10, "ki_z": 5, "kd_z": 5}
        
        # Horizontal PID control
        x_error = act_pos[0] - pos_desired[0]
        x_deriv = (x_error - self.past_x_error) / dt
        y_error = act_pos[1] - pos_desired[1]
        y_deriv = (y_error - self.past_y_error) / dt
        # PD controller
        vx_desired = 1 * np.clip(x_error, -1, 1) + 0 * x_deriv
        vy_desired = 1 * np.clip(y_error, -1, 1) + 0 * y_deriv
        vx_desired = np.clip(vx_desired, -1, 1)
        vy_desired = np.clip(vy_desired, -1, 1)
        self.past_x_error = x_error
        self.past_y_error = y_error

    
        # Velocity PID control
        vx_error = vx_desired - actual_vx
        vx_deriv = (vx_error - self.past_vx_error) / dt
        vy_error = vy_desired - actual_vy
        vy_deriv = (vy_error - self.past_vy_error) / dt
        vx_pid = gains["kp_vel_xy"] * np.clip(vx_error, -1, 1) + gains["kd_vel_xy"] * vx_deriv
        vy_pid = gains["kp_vel_xy"] * np.clip(vy_error, -1, 1) + gains["kd_vel_xy"] * vy_deriv
        A_inv = np.array([[math.sin(act_yaw), -math.cos(act_yaw)],
                          [math.cos(act_yaw), math.sin(act_yaw)]])
        desired_rp = -1*(1/ 9.8) * np.dot(A_inv, np.array([vx_pid, vy_pid]))
        desired_roll = desired_rp[0]
        desired_pitch = desired_rp[1]
        self.past_vx_error = vx_error
        self.past_vy_error = vy_error

        # Altitude PID control
        alt_error = pos_desired[2] - act_pos[2]
        alt_deriv = (alt_error - self.past_alt_error) / dt
        self.altitude_integrator += alt_error * dt
        alt_command = gains["kp_z"] * alt_error + gains["kd_z"] * alt_deriv + \
            gains["ki_z"] * np.clip(self.altitude_integrator, -2, 2) + 48
        self.past_alt_error = alt_error

        # Attitude PID control
        pitch_error = desired_pitch - act_pitch
        pitch_error = np.clip(pitch_error, -1, 1)
        pitch_deriv = (pitch_error - self.past_pitch_error) / dt
        roll_error = desired_roll - act_roll
        roll_error = np.clip(roll_error, -1, 1)
        roll_deriv = (roll_error - self.past_roll_error) / dt
        yaw_error = angle_desired[2] - act_yaw
        yaw_deriv = (yaw_error - self.past_yaw_error) / dt
        desired_yaw_rate = 1 * np.clip(yaw_error, -1, 1) + 0 * yaw_deriv
        
        yaw_rate_error = desired_yaw_rate - act_yaw_rate
        roll_rate_error = roll_error - act_roll_rate
        pitch_rate_error = pitch_error - act_pitch_rate
        
        roll_rate_deriv = (roll_rate_error - self.past_roll_rate_error) / dt
        pitch_rate_deriv = (pitch_rate_error - self.past_pitch_rate_error) / dt
        yaw_rate_deriv = (yaw_rate_error - self.past_yaw_rate_error) / dt
        
        roll_command = gains["kp_att_rp"] * np.clip(roll_rate_error, -1, 1) + 0.1 * self.roll_rate_integrator + gains["kd_att_rp"] * roll_rate_deriv
        pitch_command = -gains["kp_att_rp"] * np.clip(pitch_rate_error, -1, 1) - 0.1 * self.pitch_rate_integrator - gains["kd_att_rp"] * pitch_rate_deriv
        # pitch_command = gains["kp_att_rp"] * np.clip(pitch_rate_error, -1, 1) + 0.1 * self.pitch_rate_integrator + gains["kd_att_rp"] * pitch_rate_deriv
        yaw_command = gains["kp_att_y"] * np.clip(yaw_rate_error, -1, 1) + 0 * self.yaw_rate_integrator + 0.1* yaw_rate_deriv
        
        # Update
        self.past_pitch_error = pitch_error
        self.past_roll_error = roll_error
        self.past_roll_rate_error = roll_rate_error
        self.past_pitch_rate_error = pitch_rate_error
        self.past_yaw_rate_error = yaw_rate_error
        self.pitch_rate_integrator += pitch_rate_error * dt
        self.yaw_rate_integrator += yaw_rate_error * dt
        self.roll_rate_integrator += roll_rate_error * dt
        
        # Attitude PID control
        # pitch_error = desired_pitch - act_pitch
        # pitch_deriv = (pitch_error - self.past_pitch_error) / dt
        # roll_error = desired_roll - act_roll
        # roll_deriv = (roll_error - self.past_roll_error) / dt
        # yaw_rate_error = 0 - act_yaw_rate
        # roll_command = gains["kp_att_rp"] * np.clip(roll_error, -1, 1) + gains["kd_att_rp"] * roll_deriv
        # pitch_command = -gains["kp_att_rp"] * np.clip(pitch_error, -1, 1) - gains["kd_att_rp"] * pitch_deriv
        # yaw_command = gains["kp_att_y"] * np.clip(yaw_rate_error, -1, 1)
        # self.past_pitch_error = pitch_error
        # self.past_roll_error = roll_error


        # Motor mixing
        m1 = alt_command - roll_command + pitch_command + yaw_command
        m2 = alt_command - roll_command - pitch_command - yaw_command
        m3 = alt_command + roll_command - pitch_command + yaw_command
        m4 = alt_command + roll_command + pitch_command - yaw_command

        # Limit the motor command
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4]