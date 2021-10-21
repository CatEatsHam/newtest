#!/usr/bin/env python3

import rospy
#import tf # does not work for Python3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

import gym
from abc import abstractmethod
import numpy as np
import math
import random

def reset_gazebo():
    rospy.wait_for_service('/gazebo/reset_simulation')
    reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    try:
        reset()
    except rospy.ServiceException as ex:
        rospy.logwarn('Gazebo reset unsuccessful: ' + str(ex))
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

def euler_from_quaternion(x, y, z, w):
    '''
    Source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/

    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    '''
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # radians

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
    def reset_other(self):
        '''
        Performs remaining environment reset. Returns initial observation.
        '''
        raise NotImplementedError
    
    def reset(self):
        reset_gazebo()
        return self.reset_other()

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
            square_distance=5.0, goal_radius=0.5, timestep=0.1):
        super(MobileRobotEnv, self).__init__(command_queue)

        metadata = {'render.modes': ['console']}

        self.max_episode_steps = max_episode_steps
        self.bound = bound # meters
        self.square_distance = square_distance # meters
        self.goal_radius = goal_radius # meters
        self.timestep = timestep # seconds

        # Continuous observation space
        self.observation_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(3,),
                dtype=np.float32)
        
        # Discrete action space
        self.actions = {}
        lx = np.linspace(0.0, 1.4, 7) # no reverse
        az = np.linspace(-1.0, 1.0, 7)
        '''
        lx = np.arange(0.0, 1.0, 0.2)
        lx = np.append(lx, -lx[1:]) # include negative actions
        az = np.arange(0.0, 1.0, 0.2)
        az = np.append(az, -az[1:]) # include negative actions
        '''
        self.actions['lx'] = lx
        self.actions['az'] = az
        rospy.loginfo('action space: lx = %s az = %s' %(lx, az))
        self.action_space = gym.spaces.MultiDiscrete([lx.shape[0], az.shape[0]])

        self.goal = np.zeros(2) # x, y
        self.target_heading = 0.0
        self.pose = np.zeros(3) # x, y, yaw
        self.action = np.zeros(2) # lx, az

    def observe(self):
        # distance error
        error_dist_norm = (self.pose[:2] - self.goal) / (self.square_distance +
                self.bound)
        
        # heading error
        error_yaw = heading_error(self.target_heading, self.pose[2])
        error_yaw_norm = error_yaw / np.pi
        
        obs = np.append(error_dist_norm, error_yaw_norm)
        return obs

    def reset_other(self):
        # Choose goal as random point on square
        p1 = self.square_distance * random.uniform(-1.0, 1.0)
        p2 = self.square_distance * random.choice([-1.0, 1.0])
        self.goal = random.choice([np.array([p1, p2]), np.array([p2, p1])])
        self.target_heading = math.atan2(self.goal[1], self.goal[0])
        rospy.loginfo('goal: %s' %self.goal)

        # Initial observation
        obs = self.observe()

        self.episode_steps = 0

        return obs

    def set_goal(self, goal):
        self.goal = np.array(goal)
        self.target_heading = math.atan2(self.goal[1], self.goal[0])
        rospy.loginfo('goal: %s' %self.goal)

    def step(self, action):
        # Set up service proxies
        rospy.wait_for_service('/gazebo/pause_physics')
        pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        rospy.wait_for_service('/gazebo/unpause_physics')
        unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
     
        # Take action
        lx = self.actions['lx'][action[0]]
        az = self.actions['az'][action[1]]
        self.queue.put([lx, az]) # use command publisher
        '''
        msg = Twist()
        msg.linear.x = lx
        msg.angular.z = az
        self.cmd_pub.publish(msg)
        '''
        try:
            unpause() # unpause Gazebo physics
            rospy.sleep(self.timestep) # run physics for a timestep
            pause() # pause Gazebo physics
        except (rospy.exceptions.ROSTimeMovedBackwardsException,
                rospy.ServiceException):
            rospy.logdebug('Caught Gazebo exception')

        self.episode_steps += 1
        
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
            done = True
            reward = -1.0
            self.stop_robot()
            return obs, reward, done, info

        # Check if robot out of bounds
        if abs(self.pose[0]) > self.bound or abs(self.pose[1]) > self.bound:
            rospy.loginfo('Robot out of bounds')
            done = True
            reward = -1.0
            self.stop_robot()
            return obs, reward, done, info

        # Check if goal reached
        if distance(self.pose, self.goal) < self.goal_radius:
            rospy.loginfo('Goal reached')
            done = True
            reward = 10.0
            self.stop_robot()
            return obs, reward, done, info

        # Normal step
        return obs, reward, done, info

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.pose = np.array([msg.pose.pose.position.x,
            msg.pose.pose.position.y, yaw])
