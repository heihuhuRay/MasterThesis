#!/usr/bin/env python
# -*- coding: utf-8 -*-
# __date__ = 20180411
# created by: Ray
import sim_control
import time
import pandas as pd
import numpy as np


gamma = 0.9   #reward_decay=0.9
lr = 0.01     #learning_rate=0.01
epsilon = 0.9 #e_greedy=0.9

# state can be the max_angle_x, but there is a mapping between
# state_index:   0,    1,    2,    3,    4,    5,    6 
alpha_hip = 0
state_list =  [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]
reward_list = [   0,    0,    1,    0,    0,    0,   -1]
action_list = ['alpha_hip_increase', 'alpha_hip_reduce']
Q_table = pd.DataFrame(np.zeros((7,2)), index=state_list, columns=action_list, dtype=np.float64)
state_sum = len(state_list)

def get_next_state_index(current_state_index, action):
    '''
    input   current_state_index: int
            action: string
    output  next_state_index: int [0,6]
    '''
    if action = 'alpha_hip_increase':
        if current_state_index <= (state_sum-1):
            next_state_index = current_state_index+1
        if current_state_index = state_sum: # the last state
            next_state_index = current_state_index
    if action = 'alpha_hip_reduce':
        if current_state_index >=1:
            next_state_index = current_state_index-1
        if current_state_index = 0:
            next_state_index = current_state_index
    return next_state_index

def execute_action(action):
    if action == 'alpha_hip_increase':
        alpha_hip += 0.01
    if action == 'alpha_hip_reduce':
        alpha_hip -= 0.01

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
        q_new = reward + gamma * Q_table.loc[next_state, :].max()
    else:
        q_new = reward  # next state is terminal
    Q_table.loc[current_state, action] += lr * (q_new - q_current)  # update

def train():
    '''run 100 experiments'''
    for episode inrange(100):
        ''''restart simulator'''
        sim_control.stop_sim()
        time.sleep(3)
        sim_control.start_sim()
        '''pick a random state from the state list'''
        current_state_index = random.randint(0, state_sum)
        current_state = state_list[current_state_index]
        '''for each experiment'''
        while True:
            # choose action based on current_state
            action = choose_action(current_state)
            execute_action(action)
            # take action and get next observation and reward
            '''look up in get_next_state'''
            next_state_index = get_next_state_index()
            '''check reward list'''
            reward = reward_list[next_state_index]
            '''update Q-table'''
            #RL.learn(str(observation), action, reward, str(observation_))
            update_Q_table()

            # swap observation
            observation = observation_

            # break while loop when end of this episode
            if done:
                break

    # end of game
    print('game over')
    env.destroy()

if __name__ == "__main__":
    env = Maze()
    RL = QLearningTable(actions=list(range(env.n_actions)))

    env.after(100, train)
    env.mainloop()