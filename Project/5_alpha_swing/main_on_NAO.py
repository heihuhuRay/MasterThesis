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
from phase_diagram import *

#sensor_xyz = [robo_orientaion_x, robo_orientaion_y, robo_orientaion_z]


# def check_reward(next_state_index):
#     reward = reward_list[next_state_index]
#     if_done = is_next_state_done[next_state_index]
#     return reward, if_done



def train():
    phrase_xyz = []

    phrase_x = []
    phrase_y = []
    phrase_z = []
    TextObj.say('Attention, Fall Manager is Disabled.')
    movObj.setFallManagerEnabled(False) # True False
    go_to_init_pos()
    # Disable Fall Manager
    release_arm_stiffness() 
    TextObj.say('Please hold my wrist.')
    time.sleep(4)
    '''run 100 experiments'''
    for episode in range(10):
        release_arm_stiffness()
        print()
        print('------------------------------------')
        print('-----------episode No.', episode, '-----------')
        print('------------------------------------')
        TextObj.say('test'+ str(episode))

        # pick a random state from the state list
        current_state_index = random.randint(0, num_state)
        alpha_groups = into_list(state_dict[current_state_index]) # this is a dict, not list
        '''for each experiment'''
        k = 0
        while k<3:
            TextObj.say('K'+str(k))
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
            curr_alpha_groups = alpha_groups
            action_groups = []
            action_groups = choose_action(current_state_index, total_Q_table)#after action, should calc state rather than alpha

            # 2, take action, calc next_state
            #print('current_state_index = ', current_state_index)
            next_state_index, new_alpha_groups = get_next_state_and_new_alpha(current_state_index, action_groups, curr_alpha_groups)

            #TODO not run on NAO for now
            #swing_in_Vrep(alpha_hip) # execute the new alpha_hip in Vrep
            mean_loop_sensor, phrase_xyz = swing_on_Nao(new_alpha_groups, 340)
            print('______phrase_xyz________:', phrase_xyz)
            phrase_x = phrase_x + phrase_xyz[0]
            phrase_y += phrase_xyz[1]
            phrase_z += phrase_xyz[2]
            TextObj.say('ready to go to initial position')
            time.sleep(2)
            go_to_init_pos()
            #go_to_init_pos()
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
                action = action_groups[i]
                q_table = total_Q_table[i]
                update_Q_table(if_done, q_table, current_state_index, next_state_index, action, reward)
            #print('Q_table', Q_table)
            #print('next_state_index ____2', next_state_index)
            current_state_index = next_state_index
            curr_alpha_groups = new_alpha_groups
            # break while loop when end of this episode
            if if_done:
                break
        #plot_wrist_sensor(wrist_sensor_list, 'Wrist_Sensor'+str(episode))
    save_fig_wrist_sensor(wrist_sensor_list, 'Wrist_Sensor', episode)
    
    # save_fig_JointCommandSensor(All_Command,All_Sensor,L_HIP_ROLL,'L_HIP_ROLL',episode)
    # save_fig_JointCommandSensor(All_Command,All_Sensor,L_ANKLE_ROLL,'L_ANKLE_ROLL',episode)
    # save_fig_JointCommandSensor(All_Command,All_Sensor,R_HIP_ROLL,'R_HIP_ROLL',episode)
    # save_fig_JointCommandSensor(All_Command,All_Sensor,R_ANKLE_ROLL,'R_ANKLE_ROLL',episode)
    # save_fig_JointCommandSensor(All_Command,All_Sensor,R_HIP_PITCH,'R_HIP_PITCH',episode)
    # save_fig_JointCommandSensor(All_Command,All_Sensor,L_HIP_PITCH,'L_HIP_PITCH',episode)
    # save_fig_JointCommandSensor(All_Command,All_Sensor,L_KNEE_PITCH,'L_KNEE_PITCH',episode)
    # save_fig_JointCommandSensor(All_Command,All_Sensor,R_KNEE_PITCH,'R_KNEE_PITCH',episode)
    # save the last q table 
    total_Q_table[0].to_csv('ankle_roll_q_table.csv', index = False)
    total_Q_table[1].to_csv('knee_pitch_q_table.csv', index = False)
    total_Q_table[2].to_csv('hip_roll_q_table.csv', index = False)
    total_Q_table[3].to_csv('hip_pitch_q_table.csv', index = False)
    # end of game
    cal_derivities(phrase_x, 0.015)
    cal_derivities(phrase_y, 0.015)
    cal_derivities(phrase_z, 0.015)
    print('game over')
    


if __name__ == "__main__":
    train()