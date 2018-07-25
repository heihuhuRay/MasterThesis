# where to defin the alphas for each joints
    alpha is defined in PF_Layer, RG_Layer is defined before that，because RG Layer is kept the same in the experiments
#### PF layer define the alpha here
    myCont[L_ANKLE_ROLL].fSetPatternPF(PF_AnkleRoll)
    myCont[R_ANKLE_ROLL].fSetPatternPF(PF_AnkleRoll)

    myCont[L_KNEE_PITCH].fSetPatternPF(PF_KneePitch)
    myCont[R_KNEE_PITCH].fSetPatternPF(PF_KneePitch)

    myCont[L_HIP_ROLL].fSetPatternPF(PF_HipRoll)
    myCont[R_HIP_ROLL].fSetPatternPF(PF_HipRoll)

    myCont[L_HIP_PITCH].fSetPatternPF(PF_HipPitch)
    myCont[R_HIP_PITCH].fSetPatternPF(PF_HipPitch)
```
-------- Good Result from Sid --------

alpha_ankel_roll = 0.042
alpha_ankel_pitch = 0

alpha_knee_pitch = 0.011

alpha_hip_roll = 0.02
alpha_hip_pitch = 0.022

```

```python
base_dict example
base_dict = {
                'a_ankle_roll': 0.01,
                'a_ankle_pitch':   0,

                'a_knee_pitch': 0.01,

                'a_hip_roll':   0.01,
                'a_hip_pitch':  0.01,
            }

```


```
action_list = [ 
                'ankle_roll++',  'ankle_roll--',
                'ankle_pitch++', 'ankle_pitch--',

                'hip_roll++',    'hip_roll--',
                'hip_pitch++',   'hip_pitch--',

                'knee_pitch++',  'knee_pitch--',
            ]
```