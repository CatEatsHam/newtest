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
    save_callback = SaveCallback(10000, log_dir, save_model_path, verbose=True)
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)
    #model = PPO.load(save_model_path, env) # load from zip file
    model.learn(total_timesteps=timesteps, callback=save_callback)
    
    # Save final model
    final_model_path = model_path + '_final'
    model.save(final_model_path)

def get_goals(square_distance=4.0, episodes=50):
    perimeter = 4 * 2 * square_distance
    locations = np.linspace(0.0, perimeter, num=episodes, endpoint=False)
    corners = [perimeter / 8, 3 * perimeter/ 8, 5 * perimeter / 8, 7 *
            perimeter/ 8]
    middles = [perimeter / 4, perimeter/ 2, 3 * perimeter / 4, perimeter]
    goals = []
    for loc in locations:
        if loc < corners[0]:
            goals.append((square_distance, loc))
        elif loc < corners[1]:
            goals.append((middles[0] - (loc - corners[0]), square_distance))
        elif loc < corners[2]:
            goals.append((-square_distance, middles[1] - (loc - corners[1])))
        elif loc < corners[3]:
            goals.append((-(middles[2] - (loc - corners[2])), -square_distance))
        else:
            goals.append((square_distance, -(perimeter - loc)))
    return goals

def evaluate(env, model_path, episodes=50):
    model = PPO.load(model_path)
    episode_rewards = []
    goals = get_goals()
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

    env = MobileRobotEnv(command_queue)
    env.ros_init()
    #env = ModelEnv() # Akshay's model environment
    env.seed(0) # set seed
    env = Monitor(env, log_dir)

    #train(env, timesteps, model_path, log_dir)
    evaluate(env,
        "/home/yaggi/rl-local-planner/gym_gazebo/models/ppo_pause_final.zip")
    command_publisher_process.join()
