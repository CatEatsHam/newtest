#!/usr/bin/env python3

import rospy
import sys
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

from envs import MobileRobotEnv
from callbacks import SaveCallback

'''
Create model and train
'''
def train(env, timesteps, model_path, log_dir):
    # Set up and run training
    save_model_path = model_path + '_best'
    save_callback = SaveCallback(1000, log_dir, save_model_path, verbose=True)
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)
    model.learn(total_timesteps=timesteps, callback=save_callback)
    
    # Save final model
    final_model_path = model_path + '_final'
    model.save(final_model_path)

if __name__ == '__main__':
    rospy.init_node('train', anonymous=True)
    
    log_dir = rospy.get_param('~log_dir', './logs')
    model_dir = rospy.get_param('~model_dir', './models')
    model_name = rospy.get_param('~model_name', 'model')
    timesteps = rospy.get_param('~timesteps', 10000)
    model_path = model_dir + '/' + model_name

    env = MobileRobotEnv()
    env.seed(0) # set seed
    env = Monitor(env, log_dir)
     
    train(env, timesteps, model_path, log_dir)
    rospy.spin()
