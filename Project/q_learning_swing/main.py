#!/usr/bin/env python
# -*- coding: utf-8 -*-
# __date__ = 20180411
# created by: Ray
import sim_control
import time
import pandas as pd
import numpy as np
# state can be the max_angle_x, but there is a mapping between
# state_index:   0,    1,    2,    3,    4,    5,    6 
state_list =  [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]
reward_list = [   0,    0,    1,    0,    0,    0,   -1]
action_list = ['alpha_hip_increase', 'alpha_hip_reduce']
Q-table = pd.DataFrame(index=state_list, columns=actaction_listions, dtype=np.float64)
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

    '''run 100 experiments'''
    for episode inrange(100):
        # initial observation
        ''''restart simulator'''
        sim_control.stop_sim()
        time.sleep(3)
        sim_control.start_sim()
        
        '''pick a random state from the state list'''
        current_state_index = random.randint(0, state_sum)
        current_state = state_list[current_state_index]

        '''for each experiment'''
        while True:
            # fresh env
            # env.render()

            # RL choose action based on observation
            # action = RL.choose_action(str(observation))
            #TODO
            action = choose_action(current_state)
            # RL take action and get next observation and reward
            # observation_, reward, done = env.step(action)
            '''look up in get_next_state'''
            next_state_index = get_next_state_index()
            '''check reward list'''
            reward = reward_list[next_state_index]

            # RL learn from this transition
            '''update Q-table'''
            RL.learn(str(observation), action, reward, str(observation_))

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