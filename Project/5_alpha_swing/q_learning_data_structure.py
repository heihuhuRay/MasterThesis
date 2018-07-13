import pandas as pd
import numpy as np
import math
import pprint
import os
from decimal import Decimal

# Selectable range of alphas
available_alpha_para = [0.01, 0.02, 0.03]
max_alpha = available_alpha_para[-1]
min_alpha = available_alpha_para[0]
ankle_roll_para = available_alpha_para
knee_pitch_para = available_alpha_para
hip_roll_para   = available_alpha_para
hip_pitch_para  = available_alpha_para


gamma = 0.9   #reward_decay=0.9
lr = 0.01     #learning_rate=0.01
epsilon = 0.9 #e_greedy=0.9

single_alpha_options = len(ankle_roll_para)

state_dict = {}

# alpha_groups = [0.02, 0.03, 0.01, 0.01]
def alpha_groups_to_state_index(alpha_groups, single_alpha_options):
    a_0 = ankle_roll_para.index(alpha_groups[0])
    a_1 = knee_pitch_para.index(alpha_groups[1])
    a_2 = hip_roll_para.index(alpha_groups[2])
    a_3 = hip_pitch_para.index(alpha_groups[3])
    
    n = single_alpha_options
    # improve the extendbility 
    state_index = a_0*math.pow(n, 3) + a_1*math.pow(n, 2) + a_2*math.pow(n, 1) + a_3*math.pow(n, 0)
    #state_index = a_0*27 + a_1*9 + a_2*3 + a_3
    return state_index


for a_ankle_roll in ankle_roll_para:
    for a_knee_pitch in knee_pitch_para:
        for a_hip_roll in hip_roll_para:
            for a_hip_pitch in hip_pitch_para:
                # because dic is mutable variable, need to empty it here
                base_dict = {}

                base_dict['a_hip_pitch'] = a_hip_pitch
                base_dict['a_hip_roll'] = a_hip_roll
                base_dict['a_knee_pitch'] = a_knee_pitch
                base_dict['a_ankle_roll'] = a_ankle_roll

                l = [a_ankle_roll, a_knee_pitch, a_hip_roll, a_hip_pitch]
                num = alpha_groups_to_state_index(l, single_alpha_options)
                state_dict[num] = base_dict



# How many states in total is defined by available joint alpha values
# here is 3^3=81 state index_list [0, 80]
num_state = len(state_dict.keys())
index_list = state_dict.keys()

if os.path.exists('ankle_roll_q_table.csv'):
    ankle_roll_Q_table = pd.read_csv('ankle_roll_q_table.csv')
else:
    print('First time run this problem, if not first time, then Error!')
    ankle_roll_Q_table = pd.DataFrame(np.zeros((num_state,2)), index=index_list, columns=['ankle_roll++','ankle_roll--'], dtype=np.float64)

if os.path.exists('knee_pitch_q_table.csv'):
    knee_pitch_Q_table = pd.read_csv('knee_pitch_q_table.csv')
else:
    print('First time run this problem, if not first time, then Error!')
    knee_pitch_Q_table = pd.DataFrame(np.zeros((num_state,2)), index=index_list, columns=['knee_pitch++','knee_pitch--'], dtype=np.float64)

if os.path.exists('hip_roll_q_table.csv'):
    hip_roll_Q_table = pd.read_csv('hip_roll_q_table.csv')
else:
    print('First time run this problem, if not first time, then Error!')
    hip_roll_Q_table   = pd.DataFrame(np.zeros((num_state,2)), index=index_list, columns=['hip_roll++',  'hip_roll--'], dtype=np.float64)

if os.path.exists('hip_pitch_q_table.csv'):
    hip_pitch_Q_table = pd.read_csv('hip_pitch_q_table.csv')
else:
    print('First time run this problem, if not first time, then Error!')
    hip_pitch_Q_table  = pd.DataFrame(np.zeros((num_state,2)), index=index_list, columns=['hip_pitch++', 'hip_pitch--'], dtype=np.float64)

# action_groups: ['ankle_roll+/-', 'knee_pitch+/-', 'hip_roll+/-', 'hip_pitch+/-']
total_Q_table = [ankle_roll_Q_table, knee_pitch_Q_table, hip_roll_Q_table, hip_pitch_Q_table]

def choose_action(current_state_index, total_Q_table):
    '''
    input   current_state_index: [0, 1, ...., 79, 80]
            total_Q_table: [ankle_roll_Q_table, knee_pitch_Q_table, hip_roll_Q_table, hip_pitch_Q_table]
    output  action_groups: ['ankle_roll+/-', 'knee_pitch+/-', 'hip_roll+/-', 'hip_pitch+/-']
    '''
    action_groups = []
    for q_tab in total_Q_table:
        if np.random.uniform() < epsilon:
            state_action = q_tab.loc[current_state_index, :]
            state_action = state_action.reindex(np.random.permutation(state_action.index))     # some actions have same value
            action = state_action.idxmax()  # choose best action
            action_groups.append(action)
        else:
            action_list = []
            action_list = q_tab.columns.values.tolist()
            action = np.random.choice(action_list) # choose random action from action_list
            action_groups.append(action)
    return action_groups

state_list = index_list

end_state_index = num_state - 1

def get_next_state_and_new_alpha(current_state_index, action_groups, curr_alpha_groups):
    '''
    input   current_state_index: [0, 1, ...., 79, 80]
            action_groups:       ['ankle_roll++', 'knee_pitch++', 'hip_roll--', 'hip_pitch++']
            curr_alpha_groups:   [0.02, 0.03, 0.01, 0.01]
                                 [a_ankle_roll, a_knee_pitch, a_hip_roll, a_hip_pitch]
    output  next_state_index:    [0, 1, ...., 79, 80]
            new_alpha_groups:    [0.01, 0.03, 0.01, 0.02]
    '''
    tmp_action_groups = []
    for action in action_groups:
        action = action[-2:]
        tmp_action_groups.append(action)
    print('tmp_action_groups:', tmp_action_groups)
    print('curr_alpha_groups:', curr_alpha_groups)

    for i in range(4):
        if tmp_action_groups[i] == '++':
            curr_alpha_groups[i] = float( Decimal(str(curr_alpha_groups[i])) + Decimal('0.01') )
        if tmp_action_groups[i] == '--':
            curr_alpha_groups[i] = float( Decimal(str(curr_alpha_groups[i])) - Decimal('0.01') )
    print('1:', curr_alpha_groups)
    
    new_alpha_groups = []
    for new_alpha in curr_alpha_groups:
        if new_alpha > max_alpha:
            new_alpha = max_alpha
            new_alpha_groups.append(new_alpha)
        elif new_alpha < min_alpha:
            new_alpha = min_alpha
            new_alpha_groups.append(new_alpha)
        else:
            new_alpha_groups.append(new_alpha)
    print('2:', new_alpha_groups)
    
    next_state_index = alpha_groups_to_state_index(new_alpha_groups, single_alpha_options)
    
    return next_state_index, new_alpha_groups


def check_reward(next_state_index, mean_loop_sensor):
    #TODO calc the reward
    reward = 100 - mean_loop_sensor/100
    #reward = reward_list[next_state_index]
    if reward < 25:
        if_done = True
    else:
        if_done = False
    return reward, if_done

def update_Q_table(if_done, q_table, current_state_index, next_state_index, action, reward):
    print('reward', reward)
    #current_state = state_list[current_state_index]
    #next_state = state_list[next_state_index]
    q_current = q_table.loc[current_state_index, action]
    
    if if_done == False: # if current state is not terminal state
        # the very key point of Q-learning, how q_value is updated
        q_new = gamma * q_table.loc[next_state_index, :].max()
    else:
        q_new = 0  # next state is terminal
    q_table.loc[current_state_index, action] += lr * (reward + q_new - q_current)  # update

def into_list(origin_dict):
    '''
    input   origin_dict:  { 'a_hip_roll': 0.01,
                            'a_knee_pitch': 0.03,
                            'a_ankle_roll': 0.01,
                            'a_hip_pitch': 0.01 }
    output  list        : [0.01, 0.03, 0.01, 0.01]
                          [a_ankle_roll, a_knee_pitch, a_hip_roll, a_hip_pitch]
    '''
    alpha_list = [None, None, None, None]
    alpha_list[0] = a_ankle_roll
    alpha_list[1] = a_knee_pitch
    alpha_list[2] = a_hip_roll
    alpha_list[3] = a_hip_pitch
    return alpha_list