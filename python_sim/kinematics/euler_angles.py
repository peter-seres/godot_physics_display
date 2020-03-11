import numpy as np
import kinematics.unitquaternion as quat


class EulerAngles:
    def __init__(self, roll=None, pitch=None, yaw=None, deg=False):
        if roll is None:
            roll = 0.0
        if pitch is None:
            pitch = 0.0
        if yaw is None:
            yaw = 0.0
        if deg:
            roll *= np.pi / 180
            pitch *= np.pi / 180
            yaw *= np.pi / 180

        self.__roll = roll
        self.__pitch = pitch
        self.__yaw = yaw
        self.deg = deg

    def __repr__(self):
        return "EulerAngles[deg](roll = {:}, pitch = {:}, yaw = {:})".format(self.__roll * 180 / np.pi,
                                                                             self.__pitch * 180 / np.pi,
                                                                             self.__yaw * 180 / np.pi)

    # Class constructors
    @staticmethod
    def from_vector(v):
        """ Generate EulerAngles object from 3D vector. """

        return EulerAngles(v[0], v[1], v[2])

    @staticmethod
    def from_rotmat(R):
        """ Generate EulerAngles object from RotationMatrix """

        from .rotation_matrix import RotationMatrix
        if type(R) is RotationMatrix:
            R = R.as_matrix()
        else:
            raise ValueError("Input R must be of type RotationMatrix.")

        phi = np.arctan(R[2, 1] / R[2, 2])
        theta = -np.arcsin(R[2, 0])
        psi = np.arctan(R[1, 0] / R[0, 0])

        return EulerAngles(phi, theta, psi)

    @staticmethod
    def from_quaternion(quaternion):
        """ Generate euler angles from unit quaternion. """

        if type(quaternion) is quat.UnitQuaternion:
            q = quaternion.as_vector()
        elif type(quaternion) is np.array:
            q = quaternion
        else:
            raise TypeError('Input has to be 4D vector or quaternion.')

        roll_atan_first = 2 * (q[0] * q[1] + q[2] * q[3])
        roll_atan_second = 1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2)
        yaw_atan_first = 2 * (q[0] * q[3] + q[1] * q[2])
        yaw_atan_second = 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2)
        pitch_arcsin = 2 * (q[0] * q[2] - q[1] * q[3])

        roll = np.arctan2(roll_atan_first, roll_atan_second)
        pitch = np.arcsin(pitch_arcsin)
        yaw = np.arctan2(yaw_atan_first, yaw_atan_second)

        return EulerAngles(roll, pitch, yaw)

    # Class representations
    def as_vector(self):
        """ Represent class as a 3-by-1 vector. """

        return np.array([self.__roll, self.__pitch, self.__yaw])

    # Transformation functions
    def R_roll(self):
        """ Generate (inertial-to-body) rotation matrix (due to roll) in East-Down plane. """

        sPH = np.sin(self.__roll)
        cPH = np.cos(self.__roll)

        return np.array([1, 0, 0, 0, cPH, sPH, 0, -sPH, cPH]).reshape(3, 3)

    def R_pitch(self):
        """ Generate (inertial-to-body) rotation matrix (due to pitch) in North-Down plane. """

        sTH = np.sin(self.__pitch)
        cTH = np.cos(self.__pitch)
        return np.array([cTH, 0, -sTH, 0, 1, 0, sTH, 0, cTH]).reshape(3, 3)

    def R_yaw(self):
        """ Generate (inertial-to-body) rotation matrix (due to yaw) in North-East plane. """

        sPS = np.sin(self.__yaw)
        cPS = np.cos(self.__yaw)

        return np.array([cPS, sPS, 0, -sPS, cPS, 0, 0, 0, 1]).reshape(3, 3)

    def R_bi(self):
        """ Generate 3-by-3 rotation matrix from body to inertial frame (NED) """

        return (self.R_roll() @ self.R_pitch() @ self.R_yaw()).transpose()

    def T_eb(self):
        """ Generate 3-by-3 transformation matrix from Euler-rates to body-rates. """

        sPH = np.sin(self.__roll)
        cPH = np.cos(self.__roll)
        sTH = np.sin(self.__pitch)
        cTH = np.cos(self.__pitch)

        return np.array([1, 0, -sTH, 0, cPH, sPH * cTH, 0, -sPH, cPH * cTH]).reshape(3, 3)

    def T_be(self):
        """ Generate 3-by-3 transformation matrix from body-rates to Euler-rates. """

        sPH = np.sin(self.__roll)
        cPH = np.cos(self.__roll)
        sTH = np.sin(self.__pitch)
        cTH = np.cos(self.__pitch)

        return np.array([cTH, sPH * sTH, cPH * sTH, 0, cPH * cTH, -sPH * cTH, 0, sPH, cPH]).reshape(3, 3) / cTH
