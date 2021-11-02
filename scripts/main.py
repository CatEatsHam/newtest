#!/usr/bin/env python3

import rospy
import sys
from pathlib import Path
import multiprocessing as mp
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

from envs import MobileRobotEnv
from command_publisher import CommandPublisher
#from model_envs import ModelEnv
from callbacks import SaveCallback

'''
Create model and train
'''
def train(model_path, log_dir, timesteps=500000, use_model=True):
    command_queue = mp.Queue()

    if not use_model:
        # Create and start command manager
        command_publisher = CommandPublisher(command_queue, rate=50)
        command_publisher_process = mp.Process(target=command_publisher.publish)
        command_publisher_process.start()

    # Create environment and intialize ROS
    env = MobileRobotEnv(command_queue, max_episode_steps=300, bound=3.0,
            square_distance=3.0, goal_radius=0.2, timestep=0.05,
            use_model=use_model, evaluate=False)
    if not use_model:
        env.ros_init()
    
    env.seed(0) # set seed
    env = Monitor(env, log_dir)
    
    # Filepaths
    save_model_path = model_path + '_best'
    final_model_path = model_path + '_final'

    save_callback = SaveCallback(10000, log_dir, save_model_path, verbose=True)
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)
    #model = PPO.load(final_model_path, env) # load from zip file
    model.learn(total_timesteps=timesteps, callback=save_callback)
    
    # Save final model
    model.save(final_model_path)
    
    if not use_model:
        rospy.loginfo('Training complete')
        command_publisher_process.join()
    else:
        print('Training complete')

'''
Load model and evaluate
'''
def evaluate(model_path, episodes=50):
    command_queue = mp.Queue()
        
    # Create and start command manager
    command_publisher = CommandPublisher(command_queue, rate=50)
    command_publisher_process = mp.Process(target=command_publisher.publish)
    command_publisher_process.start()

    env = MobileRobotEnv(command_queue, max_episode_steps=300, bound=5.0,
            square_distance=3.0, goal_radius=0.2, timestep=0.05, use_model=False,
            evaluate=True)
    env.ros_init()
    
    model = PPO.load(model_path)
    episode_rewards = []
    goals = env.get_eval_goals(episodes)
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
    plot_path = Path(model_path).with_suffix('.svg')
    env.plot_save(plot_path) # save plot

    total = 0.0
    for r in episode_rewards:
        total += r

    # Print stats
    rospy.loginfo('mean episode reward: %f' %(total / episodes))
    
    command_publisher_process.join()

if __name__ == '__main__':
    log_dir = rospy.get_param('rl_env/log_dir', './logs')
    model_dir = rospy.get_param('rl_env/model_dir', './models')
    model_name = rospy.get_param('rl_env/model_name', 'model')
    timesteps = rospy.get_param('rl_env/timesteps', 500000)
    action = rospy.get_param('rl_env/action', 'evaluate')
    use_model = rospy.get_param('rl_env/use_model', True)
    model_path = model_dir + '/' + model_name

    if action == 'train':
        train(model_path, log_dir, timesteps=timesteps, use_model=use_model)
    elif action == 'evaluate':
        evaluate(model_path, episodes=50)
