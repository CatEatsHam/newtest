#!/usr/bin/env python3

import rospy
#import tf # does not work for Python3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from controller_manager_msgs.srv import (SwitchController,
        SwitchControllerRequest, SwitchControllerResponse)

import gym
from abc import abstractmethod
import numpy as np
import math
import random
from scipy.spatial.transform import Rotation

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from differential_drive_model import DifferentialDriveModel

def reset_world():
    rospy.wait_for_service('/gazebo/reset_world')
    reset = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    try:
        reset()
    except rospy.ServiceException as ex:
        rospy.logwarn('Gazebo reset unsuccessful: ' + str(ex))
        return False
    return True

def reset_simulation():
    rospy.wait_for_service('/gazebo/reset_simulation')
    reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    try:
        reset()
    except rospy.ServiceException as ex:
        rospy.logwarn('Gazebo reset unsuccessful: ' + str(ex))
        return False
    return True

def switch_controller(on=True):
    rospy.wait_for_service('/controller_manager/switch_controller')
    switch = rospy.ServiceProxy('/controller_manager/switch_controller',
            SwitchController)
    controller = 'jackal_velocity_controller'
    req = SwitchControllerRequest()
    if on:
        req.start_controllers.append(controller)
    else:
        req.stop_controllers.append(controller)
    try:
        res = switch(req)
    except rospy.ServiceException as ex:
        rospy.logwarn('Controller switch unsuccessful: ' + str(ex))
        return False
    return res.ok

def timestep_gazebo(timestep):
    rospy.wait_for_service('/gazebo/pause_physics')
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

    try:
        unpause()
        rospy.sleep(timestep)
        pause()
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        rospy.logdebug('Caught ROSTimeMovedBackwardsException')
    except rospy.ServiceException as ex:
        rospy.logwarn('Gazebo timestep unsuccessful: ' + str(ex))
        return False
    return True

def distance(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

def heading_error(target, current):
    error_raw = current - target
    if abs(error_raw) > np.pi:
        error = 2 * np.pi - abs(error_raw) 
        if error_raw > 0:
            error = -error
    else:
        error = error_raw
    
    return error

class GazeboEnv(gym.Env):
    def __init__(self, command_queue):
        super(GazeboEnv, self).__init__()
        self.queue = command_queue

    def ros_init(self):
        rospy.init_node('rl_env')
        # Publishers and subscribers
        rospy.Subscriber('/gazebo/ground_truth_odom', Odometry,
                self.odom_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    @abstractmethod
    def odom_callback(self, msg):
        raise NotImplementedError

    def stop_robot(self):
        self.queue.put([0.0, 0.0]) # use command queue
        '''
        msg = Twist() # defaults to zeros
        self.cmd_pub.publish(msg)
        rospy.sleep(1) # allow robot to stop
        '''

class MobileRobotEnv(GazeboEnv):
    def __init__(self, command_queue, max_episode_steps=300, bound=10.0,
            square_distance=5.0, goal_radius=0.5, timestep=0.1, use_model=False,
            evaluate=False):
        super(MobileRobotEnv, self).__init__(command_queue)

        metadata = {'render.modes': ['console']}

        self.max_episode_steps = max_episode_steps
        self.bound = bound # meters
        self.square_distance = square_distance # meters
        self.goal_radius = goal_radius # meters
        self.timestep = timestep # seconds
        self.use_model = use_model
        self.evaluate = evaluate

        # Continuous observation space
        self.observation_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(3,),
                dtype=np.float32)
        
        # Discrete action space
        '''
        self.actions = {}
        #lx = np.linspace(0.0, 1.5, 12) # no reverse
        #az = np.linspace(-2.5, 2.5, 5)
        lx = np.linspace(0.0, 1.4, 7) # no reverse
        az = np.linspace(-1.0, 1.0, 7)
        #lx = np.arange(0.0, 1.0, 0.2)
        #lx = np.append(lx, -lx[1:]) # include negative actions
        #az = np.arange(0.0, 1.0, 0.2)
        #az = np.append(az, -az[1:]) # include negative actions
        self.actions['lx'] = lx
        self.actions['az'] = az
        rospy.loginfo('action space: lx = %s az = %s' %(lx, az))
        self.action_space = gym.spaces.MultiDiscrete([lx.shape[0], az.shape[0]])
        '''

        # Continuous action space
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,),
                dtype=np.float32)

        self.goal = np.zeros(2) # x, y
        self.pose = np.zeros(3) # x, y, yaw
        self.action = np.zeros(2) # lx, az

        # Plotting
        if self.evaluate:
            self.plot_init()

        # Model
        if self.use_model:
            self.model = DifferentialDriveModel(wheel_radius=0.098,
                    track_width=0.37559)

    def observe(self):
        # distance error
        error_distance_norm = (self.pose[:2] - self.goal) / (
                self.square_distance + self.bound)
        
        # heading error
        goal_heading = math.atan2(self.goal[1] - self.pose[1], self.goal[0] -
                self.pose[0])
        error_heading = heading_error(goal_heading, self.pose[2])
        error_heading_norm = error_heading / np.pi
        
        obs = np.append(error_distance_norm, error_heading_norm)
        return obs

    def reset(self):
        # Reset Gazebo
        if not self.use_model:
            #reset_simulation()
            reset_world()
            timestep_gazebo(0.000001) # allow odometry to reset

        else:
            self.pose = np.zeros(3)

        if self.evaluate:
            self.path = []

        # Choose goal as random point on square
        p1 = self.square_distance * random.uniform(-1.0, 1.0)
        p2 = self.square_distance * random.choice([-1.0, 1.0])
        self.goal = random.choice([np.array([p1, p2]), np.array([p2, p1])])
        self.target_heading = math.atan2(self.goal[1], self.goal[0])
        if not self.evaluate:
            rospy.loginfo('goal: %s' %self.goal)
            if self.use_model:
                print('goal: %s' %self.goal)

        # Initial observation
        obs = self.observe()

        self.episode_steps = 0

        return obs

    def set_goal(self, goal):
        self.goal = np.array(goal)
        self.target_heading = math.atan2(self.goal[1], self.goal[0])
        rospy.loginfo('goal: %s' %self.goal)

    def step(self, action):
        # Get action
        # discrete
        '''
        lx = self.actions['lx'][action[0]]
        az = self.actions['az'][action[1]]
        '''
        # continuous
        lx = 1.5 * (action[0] + 1.0) / 2.0 # [0.0, 1.5]
        az = action[1] # [-1.0, 1.0]

        # Simulate timestep
        if not self.use_model: 
            # Take action
            self.queue.put([lx, az]) # use command publisher
            '''
            msg = Twist()
            msg.linear.x = lx
            msg.angular.z = az
            self.cmd_pub.publish(msg)
            '''
            timestep_gazebo(self.timestep)
        else:
            self.pose = self.model.step(self.pose, (lx, az), self.timestep)
        self.episode_steps += 1

        if self.evaluate:
            self.path.append(self.pose[:2])
        
        # Make observation
        obs = self.observe()

        # Calculate reward
        reward = 0.0
        '''
        d = distance(self.pose, self.goal)
        reward = -1.0 * d
        '''
        
        done = False
        info = {}

        # Check if max steps reached
        if self.episode_steps > self.max_episode_steps:
            rospy.loginfo('Max episode steps reached')
            if self.use_model:
                print('Max episode steps reached')
            done = True
            reward = -1.0
            self.stop_robot()
            if self.evaluate:
                self.plot_result(False)
            return obs, reward, done, info

        # Check if robot out of bounds
        if abs(self.pose[0]) > self.bound or abs(self.pose[1]) > self.bound:
            rospy.loginfo('Robot out of bounds')
            if self.use_model:
                print('Robot out of bounds')
            done = True
            reward = -1.0
            self.stop_robot()
            if self.evaluate:
                self.plot_result(False)
            return obs, reward, done, info

        # Check if goal reached
        if distance(self.pose, self.goal) < self.goal_radius:
            rospy.loginfo('Goal reached')
            if self.use_model:
                print('Goal reached')
            done = True
            reward = 10.0
            self.stop_robot()
            if self.evaluate:
                self.plot_result(success=True)
            return obs, reward, done, info

        # Normal step
        return obs, reward, done, info

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        r = Rotation.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        self.pose = np.array([msg.pose.pose.position.x,
            msg.pose.pose.position.y, yaw])

    def get_eval_goals(self, episodes=50):
        perimeter = 4 * 2 * self.square_distance
        locations = np.linspace(0.0, perimeter, num=episodes, endpoint=False)
        corners = [perimeter / 8, 3 * perimeter/ 8, 5 * perimeter / 8, 7 *
                perimeter/ 8]
        middles = [perimeter / 4, perimeter/ 2, 3 * perimeter / 4, perimeter]
        goals = []
        for loc in locations:
            if loc < corners[0]:
                goals.append((self.square_distance, loc))
            elif loc < corners[1]:
                goals.append((middles[0] - loc, self.square_distance))
            elif loc < corners[2]:
                goals.append((-self.square_distance, middles[1] - loc))
            elif loc < corners[3]:
                goals.append((-(middles[2] - loc), -self.square_distance))
            else:
                goals.append((self.square_distance, -(perimeter - loc)))
        return goals

    def plot_init(self):
        plt.ion() # interactive on
        fig, self.ax = plt.subplots()
        plt.title('Evaluation Results')
        plt.xlabel('meters')
        plt.ylabel('meters')
        self.ax.scatter(0, 0, marker='+', color='black')
        plt.draw()
        plt.pause(0.1)

    def plot_result(self, success=False, path=True):
        marker_val = 'x'
        if success:
            marker_val = 'o'
        self.ax.scatter(self.goal[0], self.goal[1], marker=marker_val)
        if path is not None:
            x = [a[0] for a in self.path]
            y = [a[1] for a in self.path]
            self.ax.plot(x, y)
        plt.draw()
        plt.pause(0.1)

    def plot_save(self, filename='plot'):
        plt.savefig(filename)
