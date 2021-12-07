# python modules
import argparse
import importlib
import math
import random
import numpy as np
import os.path
import sys
import collections
import tensorflow as tf
from tensorflow import keras

# project files
import dynamic_obstacle
import bounds
import robot # two integrator robot
import simu_env

import param

# from pe_model import PE
# from fake_env import FakeEnv
from ssa import SafeSetAlgorithm

# Display
# TODO: switch to pyglet
# https://github.com/openai/multiagent-particle-envs
from base_display import BaseDisplay
from pyglet_display import PygletDisplay

import os

def display_for_name( dname ):
    # choose none display or visual display
    # if dname == 'turtle':
    #     return TurtleRunnerDisplay(800,800)
    if dname == 'pyglet':
         return PygletDisplay(800,800)
    else:
      return BaseDisplay()


def run_kwargs( params ):
    in_bounds = bounds.BoundsRectangle( **params['in_bounds'] )
    goal_bounds = bounds.BoundsRectangle( **params['goal_bounds'] )
    min_dist = params['min_dist']
    # TODO 这里传param.py的参数
    ret = { 'field': dynamic_obstacle.ObstacleField(params['static_obstacles']),
            'robot_state': robot.DoubleIntegratorRobot( **( params['initial_robot_state'] ) ),
            'in_bounds': in_bounds,
            'goal_bounds': goal_bounds,
            'noise_sigma': params['noise_sigma'],
            'min_dist': min_dist,
            'nsteps': 1000, 'static_obs':params['static_obstacles']}
    return ret

def parser():
    prsr = argparse.ArgumentParser()
    prsr.add_argument( '--display',
                       choices=('pyglet','none'),
                       default='none' )
    prsr.add_argument( '--explore',
                   choices=('psn','rnd','none'),
                   default='none' )
    prsr.add_argument( '--qp',dest='is_qp', action='store_true')
    prsr.add_argument( '--no-qp',dest='is_qp', action='store_false')
    # prsr.add_argument( '--ssa-buffer',dest='enable_ssa_buffer', action='store_true')
    # prsr.add_argument( '--no-ssa-buffer',dest='enable_ssa_buffer', action='store_false')

    prsr.add_argument( '--mode',
                   choices=('rl','safe','human'),
                   default='rl' )
    prsr.add_argument( '--env',
                   choices=('default','cross','circle'),
                   default='default' )
    # prsr.add_argument('--human', type=bool, default=False)
    prsr.add_argument('--isHumanBuffer', type=bool, default=False)
    # prsr.add_argument('--bufferLocation', type=str, default='')
    prsr.add_argument('--saveModelCheckpointPth', type=str, default='./model_checkpoints')
    prsr.add_argument('--loadModelCheckpointPth', type=str, default='./model_checkpoints/100eps')
    prsr.add_argument('--isLoadModel', type=bool, default=False)
    prsr.add_argument('--replaceRatio', type=float, default=0.4)
    prsr.add_argument('--maxEpisode', type=int, default=501)    
    prsr.add_argument('--trainNum', type=int, default=1)
    return prsr

def main(display_name, env_name, exploration, qp, is_human_buffer, mode, is_load, \
    save_model_checkpoint_path, load_model_checkpoint_path, replace_ratio, max_episode, train_num):

    params = param.params
    display = display_for_name(display_name)
    env_params = run_kwargs(params)
    
    # rl policy
    obstacle_num = 3
    robot_state_size = 4 #(x,y,v_x,v_y)
    robot_action_size = 2
    nearest_obstacle_state_size = 2 #(delta_x, delta_y)
    state_dim = robot_state_size + nearest_obstacle_state_size * obstacle_num

    model_update_freq = 1000
    env = simu_env.Env(display, **(env_params))
    
    # ssa
    safe_controller = SafeSetAlgorithm(max_speed = env.robot_state.max_speed, is_qp = qp)
    # parameters
    max_steps = int(1e6)
    start_timesteps = 2e3
    episode_reward = 0
    episode_num = 0
    last_episode_reward = 0
    teacher_forcing_rate = 0
    total_rewards = []
    total_steps = 0
    # dynamic model parameters
    fx = np.array([[0,0,1,0],[0,0,0,1],[0,0,0,0],[0,0,0,0]])
    gx = np.array([[1,0],[0,1],[1,0],[0,1]])
    state, done = env.reset(), False
    collision_num = 0
    failure_num = 0
    success_num = 0

    is_meet_requirement = False
    reward_records = []

    for t in range(max_steps):
      action = [0, 0.01]      
      env.display_start()     
      # ssa parameters
      unsafe_obstacle_ids, unsafe_obstacles = env.find_unsafe_obstacles(env.min_dist * 6)
      action, is_safe = safe_controller.get_safe_control(state[:4], unsafe_obstacles, fx, gx, action)
      s_new, reward, done, info = env.step(action, is_safe, unsafe_obstacle_ids) 
      original_reward = reward
      episode_reward += original_reward  
      env.display_end()
      state = s_new
      if (done and original_reward == -500):          
        collision_num += 1      
      elif (done and original_reward == 2000):
        success_num += 1     
      elif (done):
        failure_num += 1
      if (done):  
        total_steps += env.cur_step
        print(f"Train: episode_num {episode_num}, total_steps {total_steps}, reward {episode_reward}, is_qp {qp}, exploration {exploration}, last state {state[:2]}")
        total_rewards.append(episode_reward)
        episode_reward = 0
        episode_num += 1
        state, done = env.reset(), False
    return reward_records


if __name__ == '__main__':
    args = parser().parse_args()
    for i in range(1):
      reward_records = main(display_name = args.display,          
          env_name = args.env,
          exploration = args.explore,
          qp = args.is_qp,
          is_human_buffer=args.isHumanBuffer,
          mode=args.mode,
          is_load=args.isLoadModel,
          save_model_checkpoint_path=args.saveModelCheckpointPth,
          load_model_checkpoint_path=args.loadModelCheckpointPth, 
          replace_ratio=args.replaceRatio,
          max_episode=args.maxEpisode,
          train_num=args.trainNum)

