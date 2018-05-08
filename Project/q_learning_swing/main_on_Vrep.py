# -*- coding: UTF-8 -*-
from __future__ import print_function

#!/usr/bin/env python
# __date__ = 20180411
# created by: Ray
# this program should be run on your laptop, not on Pi or NAO


import sim_control
import time
import pandas as pd
import numpy as np
import random
from naoqi import ALProxy
from vrep_swing_q_learning import swing_in_Vrep

PORT = 9559
robotIP = "nao.local"

try:
    memoryProxy = ALProxy("ALMemory", robotIP, PORT)
except Exception, e:
    print("Could not create proxy to ALMemory")
    print("Error was: ", e)




gamma = 0.9   #reward_decay=0.9
lr = 0.01     #learning_rate=0.01
epsilon = 0.9 #e_greedy=0.9

# state can be the max_angle_x, but there is a mapping between
state_list =         [ 0.01,  0.02, 0.03,  0.04,  0.05,  0.06, 0.07]
reward_list =        [    0,     0,    1,     0,     0,     0,  -10]
is_next_state_done = [False, False, True, False, False, False, True]
action_list = ['alpha_hip_increase', 'alpha_hip_reduce']


Q_table = pd.DataFrame(np.zeros((7,2)), index=state_list, columns=action_list, dtype=np.float64)
#print('init Q_table', Q_table)
state_sum = len(state_list) - 1
print('state_sum', state_sum)

def get_next_state_index(current_state_index, action):
    '''
    input   current_state_index: 0, 1, 2, 3, 4, 5, 6
            action: string
    output  next_state_index: int [0,6]
    '''
    if (current_state_index == 2) or (current_state_index == 6):
        # if the random init is the terminal state, then break
        # because in this situation, the q_value should update
        raise('Error: current_state is terminal state, check input source')
    else:
        if action == 'alpha_hip_increase':
            if current_state_index <= (state_sum-1):
                next_state_index = current_state_index+1
            if current_state_index == state_sum: # the last state
                next_state_index = current_state_index
        if action == 'alpha_hip_reduce':
            if current_state_index >= 1:
                next_state_index = current_state_index-1
            if current_state_index == 0:
                next_state_index = current_state_index
    return next_state_index

# No need to calc alpha, just check table
def execute_action(alpha_hip, action):
    if action == 'alpha_hip_increase':
        alpha_hip += 0.01
    if action == 'alpha_hip_reduce':
        alpha_hip -= 0.01
    return alpha_hip

def choose_action(current_state):
    if np.random.uniform() < epsilon:
        state_action = Q_table.loc[current_state, :]
        state_action = state_action.reindex(np.random.permutation(state_action.index))     # some actions have same value
        action = state_action.idxmax()  # choose best action
    else:
        action = np.random.choice(action_list) # choose random action from action_list
    return action

def update_Q_table(current_state_index, next_state_index, action, reward):
    current_state = state_list[current_state_index]
    next_state = state_list[next_state_index]
    q_current = Q_table.loc[current_state, action]
    
    if current_state_index != 6: # if current state is not terminal state
        # the very key point of Q-learning, how q_value is updated
        q_new = gamma * Q_table.loc[next_state, :].max()
    else:
        q_new = 0  # next state is terminal
    Q_table.loc[current_state, action] += lr * (reward + q_new - q_current)  # update


def check_reward(next_state_index):
    reward = reward_list[next_state_index]
    if_done = is_next_state_done[next_state_index]
    return reward, if_done

def train():
    #alpha_hip = -100
    '''run 100 experiments'''
    for episode in range(100):
        print()
        print('------------------------------------')
        print('-----------episode No.', episode, '-----------')
        print('------------------------------------')
        if episode % 10 == 0:
            print('Q_table', Q_table)
        data = memProxy.getData("WristForceSensor")
        print('WristForceSensor', data)
        # pick a random state from the state list
        current_state_index = random.randint(0, state_sum)
        current_state = state_list[current_state_index]
        alpha_hip = current_state
        '''for each experiment'''
        k = 0
        while True:
            k += 1
            # if the random init is the terminal state, then break
            # because in this situation, the q_value should update
            if (current_state_index == 2) or (current_state_index == 6):
                break
            print()
            print('############## k =', k, '###############')
            # 1, choose action based on current_state
            action = choose_action(current_state)#after action, should calc state rather than alpha

            # 2, take action, calc next_state
            #print('current_state_index = ', current_state_index)
            next_state_index = get_next_state_index(current_state_index, action)

            alpha_hip = state_list[next_state_index]
            # print('alpha_hip == next_state ==', alpha_hip)
            #TODO not run on NAO for now
            #swing_in_Vrep(alpha_hip) # execute the new alpha_hip in Vrep
            swing_on_NAO(alpha_hip)
            #time.sleep(5)

            # 3, check reward and if_done
            #print('  next_state_index =  ', next_state_index)
            reward, if_done = check_reward(next_state_index)
            print('reward', reward)
            print('if_done', if_done)

            # 4, '''update Q-table'''
            update_Q_table(current_state_index, next_state_index, action, reward)
            print('Q_table', Q_table)
            #print('next_state_index ____2', next_state_index)
            current_state_index = next_state_index
            # break while loop when end of this episode
            if if_done:
                break

    # end of game
    print('game over')


if __name__ == "__main__":
    train()