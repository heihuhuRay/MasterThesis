# -*- coding: UTF-8 -*-
from __future__ import print_function
from decimal import Decimal
#!/usr/bin/env python
# __date__ = 20180411
# created by: Ray
# this program should be run on your laptop, not on Pi or NAO


import time
import math
import pandas as pd
import numpy as np
import random
from alpha_value import *
from SetTiming import *
from NAOMotor import *
from MLMPCPG import *
from naoqi import ALProxy
from swing_on_real_Nao import *
from q_learning_data_structure import *



# def check_reward(next_state_index):
#     reward = reward_list[next_state_index]
#     if_done = is_next_state_done[next_state_index]
#     return reward, if_done



def train():
    TextObj.say('Attention, Fall Manager is Disabled.')
    movObj.setFallManagerEnabled(False) # True False
    go_to_init_pos()
    # Disable Fall Manager
    release_arm_stiffness() 
    TextObj.say('Please hold my wrist.')
    time.sleep(4)
    '''run 100 experiments'''
    for episode in range(5):
        release_arm_stiffness()
        print()
        print('------------------------------------')
        print('-----------episode No.', episode, '-----------')
        print('------------------------------------')
        TextObj.say('Experiment'+ str(episode))
        #print('Q_table', Q_table)

        # pick a random state from the state list
        current_state_index = random.randint(0, num_state)
        # check if  state_dict is empty
        print('---------------state dict----------------')
        pprint.pprint(state_dict)
        alpha_groups = state_dict[current_state_index]
        '''for each experiment'''
        k = 0
        while True:
            k += 1
            # if the random init is the terminal state, then break, because in this situation, the q_value should update
            # find a method to evaluate the boundry condition, like the robot fall down or not

            sensor_data = memProxy.getData("WristForceSensor")
            sensor_data = sum(sensor_data[0]) + sum(sensor_data[1])
            # if (sensor_data > 1000):
            #     reward = -10
            #     # update Q-table
            #     update_Q_table()
            #     break
            # if (sensor_data < 300):
            #     reward = 10
            #     break
            print()
            print('############## k =', k, '###############')
            # 1, choose action based on current_state
            action_groups = []
            action_groups = choose_action(current_state_index, total_Q_table)#after action, should calc state rather than alpha

            # 2, take action, calc next_state
            #print('current_state_index = ', current_state_index)
            next_state_index, new_alpha_groups = get_next_state_and_new_alpha(current_state_index, action_groups, alpha_groups)

            #TODO not run on NAO for now
            #swing_in_Vrep(alpha_hip) # execute the new alpha_hip in Vrep
            mean_loop_sensor = swing_on_Nao(new_alpha_groups, 400)
            print('mean_loop_sensor', mean_loop_sensor)
            #time.sleep(5)

            # 3, check reward and if_done
            #print('  next_state_index =  ', next_state_index)
            reward, if_done = check_reward(next_state_index, mean_loop_sensor)
            print('reward', reward)
            print('if_done', if_done)

            # 4, '''update Q-table'''
            for i in range(4):
                # action_groups: ['ankle_roll+/-', 'knee_pitch+/-', 'hip_roll+/-', 'hip_pitch+/-']
                # total_Q_table = [ankle_roll_Q_table, knee_pitch_Q_table, hip_roll_Q_table, hip_pitch_Q_table]
                action = action_groups[0]
                q_table = total_Q_table[0]
                update_Q_table(if_done, q_table, current_state_index, next_state_index, action, reward)
            #print('Q_table', Q_table)
            #print('next_state_index ____2', next_state_index)
            current_state_index = next_state_index
            # break while loop when end of this episode
            if if_done:
                break

    # end of game
    print('game over')
    


if __name__ == "__main__":
    train()