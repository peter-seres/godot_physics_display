import numpy as np

R_i2g = np.array([[1, 0, 0],    # North
                  [0, 0, -1],   # Up
                  [0, 1, 0]])   # East

quaternion_transform_i2g = np.array([[0, 1,  0,  0],     # q_i
                                     [0, 0,  0, -1],     # -q_z
                                     [0, 0,  1,  0],     # -q_y
                                     [1, 0,  0,  0]])    # q_real
