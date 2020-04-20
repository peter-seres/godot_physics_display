import numpy as np


def transform_translation(vec: np.ndarray) -> list:
    """ Take a vector of size 3 and converts it to Godot coordinate system: X: North, Y: Up, Z: East"""

    vec_godot = [vec[0], -vec[2], vec[1]]
    return vec_godot


def transform_rotation(quat: np.ndarray) -> list:
    """ Takes a Vector of size 4 representing a unit quaternion and converts it to Godot coordinate system: X: North, Y: Up, Z: East"""
 
    quat_godot = [quat[1], -quat[3], quat[2], quat[0]]
    return quat_godot
