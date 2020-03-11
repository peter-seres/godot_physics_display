import numpy as np


def hat(v: np.ndarray = np.zeros(3)):
    """ Skew-symmetric matrix in SO(3) from 3D vector. """
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def vex(m: np.ndarray = np.eye(3)):
    """ 3D vector from skew-symmetric matrix in SO(3). """

    return np.array([m[2, 1], m[0, 2], m[1, 0]])


def symproj(H: np.ndarray = np.eye(3)):
    """ Symmetric projection of a square matrix. """

    return (H + H.T) / 2


def asymproj(H: np.ndarray = np.eye(3)):
    """ Anti-symmetric projection of a square matrix. """

    return (H - H.T) / 2
