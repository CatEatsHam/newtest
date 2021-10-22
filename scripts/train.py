#!/usr/bin/env python3

import rospy
import sys
import multiprocessing as mp
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

from envs import MobileRobotEnv
from command_publisher import CommandPublisher
from model_envs import ModelEnv
from callbacks import SaveCallback

'''
Create model and train
'''
def train(env, timesteps, model_path, log_dir):
    # Set up and run training
    save_model_path = model_path + '_best'
    final_model_path = model_path + '_final'

    save_callback = SaveCallback(10000, log_dir, save_model_path, verbose=True)
    #model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)
    model = PPO.load(final_model_path, env) # load from zip file
    model.learn(total_timesteps=timesteps, callback=save_callback)
    
    # Save final model
    model.save(final_model_path)

def evaluate(env, model_path, episodes=50):
    model = PPO.load(model_path)
    episode_rewards = []
    goals = env.get_eval_goals()
    for goal in goals:
        done = False
        try:
            obs = env.reset()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
        env.set_goal(goal)
        episode_reward = 0
        while not done and not rospy.is_shutdown():
            action, state = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            episode_reward += reward
            if done:
                episode_rewards.append(episode_reward)

    total = 0.0
    for r in episode_rewards:
        total += r

    # Print stats
    print('mean episode reward: %f' %(total / episodes))

if __name__ == '__main__':
    command_queue = mp.Queue()

    command_publisher = CommandPublisher(command_queue, rate=50)
    
    command_publisher_process = mp.Process(target=command_publisher.publish)
    command_publisher_process.start()

    log_dir = rospy.get_param('rl_env/log_dir', './logs')
    model_dir = rospy.get_param('rl_env/model_dir', './models')
    model_name = rospy.get_param('rl_env/model_name', 'model')
    timesteps = rospy.get_param('rl_env/timesteps', 1000000)
    model_path = model_dir + '/' + model_name

    '''
    env = MobileRobotEnv(command_queue, max_episode_steps=300, bound=5.0,
            square_distance=3.0, goal_radius=0.2, timestep=0.05, use_model=True,
            evaluate=False)
    '''
    env = MobileRobotEnv(command_queue, max_episode_steps=300, bound=5.0,
            square_distance=3.0, goal_radius=0.2, timestep=0.05, use_model=False,
            evaluate=True)
    env.ros_init()
    #env = ModelEnv() # Akshay's model environment
    env.seed(0) # set seed
    env = Monitor(env, log_dir)

    #train(env, timesteps, model_path, log_dir)
    #gazebo_controller = '/home/yaggi/rl-local-planner/gym_gazebo/models/ppo_pause_final.zip'
    #model_controller = '/home/yaggi/rl-local-planner/RL_ControllersDifferentialDrive/sbmodels_jackal/yaggi_sparce.zip'
    model_controller = '/home/yaggi/rl-local-planner/gym_gazebo/models/ppo_model_fix_4M.zip'
    evaluate(env, model_controller)
    command_publisher_process.join()
