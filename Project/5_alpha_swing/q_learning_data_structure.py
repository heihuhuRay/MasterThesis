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

                state_dict[num] = base_dict
                
