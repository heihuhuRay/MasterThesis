import pandas as pd
import numpy as np
import math
import pprint

# Selectable range of alphas
ankle_roll_para = [0.01, 0.02, 0.03]
knee_pitch_para = [0.01, 0.02, 0.03]
hip_roll_para = [0.01, 0.02, 0.03]
hip_pitch_para = [0.01, 0.02, 0.03]

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

num_state = len(state_dict.keys())
index_list = state_dict.keys()

hip_roll_Q_table   = pd.DataFrame(np.zeros((num_state,2)), index=index_list, columns=['hip_roll++',  'hip_roll--'], dtype=np.float64)
hip_pitch_Q_table  = pd.DataFrame(np.zeros((num_state,2)), index=index_list, columns=['hip_pitch++', 'hip_pitch--'], dtype=np.float64)
ankle_roll_Q_table = pd.DataFrame(np.zeros((num_state,2)), index=index_list, columns=['ankle_roll++','ankle_roll--'], dtype=np.float64)
knee_pitch_Q_table = pd.DataFrame(np.zeros((num_state,2)), index=index_list, columns=['knee_pitch++','knee_pitch--'], dtype=np.float64)

state_list = index_list

end_state_index = num_state - 1

def get_next_state_index(current_state_index, action_groups, alpha_groups):
    '''
    input   current_state_index: [0, 1, ...., 79, 80]
            alpha_groups:  [0.02, 0.03, 0.01, 0.01]
            [a_hip_pitch, a_hip_roll, a_knee_pitch, a_ankle_roll]
    output  next_state_index: int [[0, 1, ...., 79, 80]
    '''
    if (current_state_index == 0) or (current_state_index == end_state_index):
        # if the random init is the terminal state, then break
        # because in this situation, the q_value should update
        raise('Error: current_state is terminal state, check input source')
    else:
        for action in alpha_groups:
            if action == 'alpha_hip++':
                if current_state_index <= (state_sum-1):
                    next_state_index = current_state_index+1
                if current_state_index == state_sum: # the last state
                    next_state_index = current_state_index
            if action == 'alpha_hip--':
                if current_state_index >= 1:
                    next_state_index = current_state_index-1
                if current_state_index == 0:
                    next_state_index = current_state_index
    return next_state_index
