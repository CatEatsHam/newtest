#!/usr/bin/env python3

import math
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import time

class DifferentialDriveModel:
    def __init__(self, wheel_radius, track_width, plot=False):
        self.wheel_radius = wheel_radius
        self.track_width = track_width
        self.plot = plot
        if plot:
            self.plot_init()

    '''
    state is x, y, theta
    '''
    def step(self, state, action, timestep=0.1, plot=False):
        lx = action[0]
        az = action[1]

        # left and right wheel angular velocities
        omega_l = lx / self.wheel_radius - az * self.track_width / (2 *
                self.wheel_radius)
        omega_r = lx / self.wheel_radius + (az * self.track_width) / (2 *
                self.wheel_radius)

        # kinematics
        x_dot = self.wheel_radius * (omega_l + omega_r) / 2 * math.cos(state[2])
        y_dot = self.wheel_radius * (omega_l + omega_r) / 2 * math.sin(state[2])
        theta_dot = self.wheel_radius * (omega_r - omega_l) / self.track_width
        state_dot = np.array([x_dot, y_dot, theta_dot])

        # calculate next state
        next_state = state + state_dot * timestep

        # limit yaw
        if next_state[2] > math.pi:
            next_state[2] -= 2 * math.pi
        elif next_state[2] < -math.pi:
            next_state[2] += 2 * math.pi

        if self.plot:
            self.plot_pose(next_state)

        return next_state
    
    def plot_init(self):
        plt.ion # interactive on
        fig, self.ax = plt.subplots()
        plt.title('Trajectory')
        plt.xlabel('meters')
        plt.ylabel('meters')
        plt.draw()
        plt.pause(0.1)

    def plot_pose(self, pose):
        x = pose[0]
        y = pose[1]
        yaw = pose[2]

        self.ax.scatter(x, y, color='black')
        self.ax.arrow(x, y, math.cos(yaw), math.sin(yaw), color='red')
        plt.draw
        plt.pause(0.1)

if __name__ == '__main__':
    ddm = DifferentialDriveModel(wheel_radius=0.1, track_width=0.3765,
            plot=True)
    pose = np.zeros(3)
    action = [1.2, -0.7]
    for i in range(100):
        pose = ddm.step(pose, action)
