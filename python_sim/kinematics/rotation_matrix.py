import numpy as np


class RotationMatrix:
    def __init__(self, R=None):
        if R is None:
            self.__R = np.eye(3)
        else:
            self.__R = R

    def __repr__(self):
        return "R in SO(3) =\n{:}".format(self.__R)

# Class representations
    def as_vector(self):
        return self.__R.reshape(9)

    def as_matrix(self):
        return self.__R

# Class constructors

    @staticmethod
    def from_vector(v):
        """ Generate rotation matrix from 9-by-1 matrix. """

        return RotationMatrix(v.reshape(3, 3))

    @staticmethod
    def from_yawZ(yaw: float = 0.0, z: np.ndarray = None):
        """ Generate rotation matrix from yaw angle and inertial Z axis commands. """

        if z is None:
            z = np.array([0.0, 0.0, 1.0])
        elif type(z) is np.ndarray:
            z /= np.linalg.norm(z)
        else:
            raise ValueError("Input Z must be numpy array is specified.")

        y = np.array([-np.sin(yaw), np.cos(yaw), 0.0])  # Desired body Y axis (East)
        x = np.cross(y, z)  # Desired body X axis (North)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)  # Recompute body Y axis (East)
        return RotationMatrix(np.hstack((x[:, None], y[:, None], z[:, None])))
