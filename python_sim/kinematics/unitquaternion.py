import numpy as np
from .so3_functions import hat, asymproj, vex
from .euler_angles import EulerAngles
from .rotation_matrix import RotationMatrix
from typing import Union


def as_matrix(q):
    """ Does not have to be UnitQuaternion. """

    if type(q) is UnitQuaternion:
        q = q.as_vector()
    elif type(q) is np.array or type(q) is np.ndarray or type(q) is list:
        q = np.array(q)

    Q = np.zeros(shape=[4, 4])

    Q[0, :] = [q[0], -q[1], -q[2], -q[3]]
    Q[1, :] = [q[1],  q[0], -q[3],  q[2]]
    Q[2, :] = [q[2],  q[3],  q[0], -q[1]]
    Q[3, :] = [q[3], -q[2],  q[1],  q[0]]

    return Q


class UnitQuaternion:
    def __init__(self, w: float = 1.0, xyz: np.ndarray = np.zeros(3)):
        """ Quaternion constructor
        >> Quaternion()
        Quaternion(1.0 + 0.0*i + 0.0*j + 0.0*k)

        >> Quaternion(4, xyz = [1,2,3])
        Quaternion(4.0 + 1.0*i + 2.0*j + 3.0*k)

        >> Quaternion(4, yxz = np.array([1,2,3]))
        Quaternion(4.0 + 1.0*i + 2.0*j + 3.0*k)
        """

        self.__q = np.array([w, xyz[0], xyz[1], xyz[2]])
        self.__normalize()

    @property
    def __w(self):
        return self.__q[0]

    @property
    def __x(self):
        return self.__q[1]

    @property
    def __y(self):
        return self.__q[2]

    @property
    def __z(self):
        return self.__q[3]

    @property
    def norm(self):
        return np.linalg.norm(self.__q)

    @property
    def is_unit(self, tolerance=1e-17):
        return abs(1.0 - self.norm) < tolerance

    @property
    def real(self):
        """ Real / Scalar part of quaternion. """
        return self.__w

    @property
    def imag(self):
        """ Imaginary / Vector part of a quaternion """
        return np.array([self.__x, self.__y, self.__z])

    def __normalize(self):
        if not self.is_unit:
            if self.norm > 0:
                self.__q = self.__q / self.norm

    def conjugate(self):
        return UnitQuaternion(self.real, -self.imag)

    def inverse(self):
        """ inverse = conjugate / norm for general quaternions. Unit quats are already normalized."""
        return UnitQuaternion(self.real, -self.imag)

    def flipped(self):
        """ The quaternion on the opposite side of the 4D sphere."""
        return UnitQuaternion(-self.real, self.imag)

    def as_vector(self):
        """ Returns the quaternion elements as a 4D numpy array. """

        return self.__q

    def as_prodmat(self):
        """
        Return quaternion product matrix (Kronecker matrix):

               np.ndarray([ [self.__w, -self.__x, -self.__y, -self.__z],
                            [self.__x,  self.__w, -self.__z,  self.__y],
                            [self.__y,  self.__z,  self.__w, -self.__x],
                            [self.__z, -self.__y,  self.__x,  self.__w]])
        """

        Q = np.eye(4) * self.real
        Q[0, 1:4] -= self.imag
        Q[1:4, 0] += self.imag
        Q[1:4, 1:4] += hat(self.imag)
        return Q

    def __quatprod(self, q):
        """ Quaternion Multiplication."""

        return UnitQuaternion.from_wxyz(self.as_prodmat() @ q.as_vector())

    def __quatrot(self, v: np.ndarray = np.ones(3)):
        """
        Rotates a 3D vector

        q.conjugated().quatrot(v) -> transforms 'v' from inertial to body frame
        q.quatrot(v) -> transforms 'v' from body to inertial frame
        """

        q_inv = self.inverse().as_vector()
        V = as_matrix(np.append(0, v))
        Q = self.as_prodmat()
        v_rot = np.matmul(Q, np.matmul(V, q_inv))   # v = Q * v * conj(q)

        return v_rot[1:]

    def unitX(self):
        """ Return X axis of respective rotation matrix (body X). """

        return np.array([self.__w ** 2 + self.__x ** 2 - self.__y ** 2 - self.__z ** 2,
                         2 * (self.__x * self.__y + self.__w * self.__z),
                         2 * (self.__x * self.__z - self.__w * self.__y)])

    def unitY(self):
        """ Return Y axis of respective rotation matrix (body Y). """

        return np.array([2 * (self.__x * self.__y - self.__w * self.__z),
                         self.__w ** 2 + self.__y ** 2 - self.__x ** 2 - self.__z ** 2,
                         2 * (self.__y * self.__z + self.__w * self.__x)])

    def unitZ(self):
        """ Return Z axis of respective rotation matrix (body Z). """

        return np.array([2 * (self.__x * self.__z + self.__w * self.__y),
                         2 * (self.__y * self.__z - self.__w * self.__x),
                         self.__w ** 2 + self.__z ** 2 - self.__x ** 2 - self.__y ** 2])

    def R_bi(self):
        """ Generate body-to-inertial rotation matrix from quaternion. """

        return np.eye(3) + 2 * self.real * hat(self.imag) + 2 * np.linalg.matrix_power(hat(self.imag), 2)

    def as_euler(self):

        """ Return EulerAngles() representation of rotation."""
        roll_atan_first = 2 * (self.__w * self.__x + self.__y * self.__z)
        roll_atan_second = 1.0 - 2.0 * (self.__x ** 2 + self.__y ** 2)
        yaw_atan_first = 2 * (self.__w * self.__z + self.__x * self.__y)
        yaw_atan_second = 1.0 - 2.0 * (self.__y ** 2 + self.__z ** 2)
        pitch_arcsin = 2 * (self.__w * self.__y - self.__x * self.__z)

        roll = np.arctan2(roll_atan_first, roll_atan_second)
        pitch = np.arcsin(pitch_arcsin)
        yaw = np.arctan2(yaw_atan_first, yaw_atan_second)

        return EulerAngles(roll, pitch, yaw)

    def as_rotation_matrix(self):
        return self.R_bi()

    def q_dot(self, omega: np.array):
        """ Returns a 4D numpy array representing the rate of the change of the quaternion. """

        omega = np.append(0, omega)  # Add a zero as the scalar part

        return 0.5 * self.as_prodmat() @ omega  # q_dot = 0.5 * Q * [0, omega]^T

    # Overrides ____________________________________________________________________________

    def __repr__(self):
        return "UnitQuaternion: w = {:}, x = {:}i, y = {:}j, z = {:}k".format(self.__w, self.__x, self.__y, self.__z)

    def __mul__(self, other):
        if type(other) is UnitQuaternion:
            return self.__quatprod(q=other)
        elif type(other) is np.array or type(other) is np.ndarray or type(other) is list:
            if len(other) != 3:
                raise ValueError("UnitQuaternion multiplication must be done on 3D Vector or another UnitQuaternion.")
            else:
                return self.__quatrot(v=np.array(other))

    def __add__(self, other):
        if type(other) is UnitQuaternion:
            return UnitQuaternion.from_wxyz(self.__q + other)
        else:
            raise ValueError("UnitQuaternion addition must be done with another UnitQuaternion.")

    def __sub__(self, other):
        if type(other) is UnitQuaternion:
            return UnitQuaternion.from_wxyz(self.__q + other)
        else:
            raise ValueError("UnitQuaternion addition must be done with another UnitQuaternion.")

    # Constructors ____________________________________________________________________________

    @staticmethod
    def from_wxyz(*wxyz):
        if len(wxyz) == 1:
            wxyz = wxyz[0]
        else:
            raise ValueError("Argument must have length 1.")
        return UnitQuaternion(w=wxyz[0], xyz=wxyz[1:4])

    # Generate quaternion corresponding to a certain roll angle (yaw = pitch = 0)
    @staticmethod
    def from_roll(eul: EulerAngles):  # e.g. EulerAngle.as_vector()
        if type(eul) == EulerAngles:
            eul = eul.as_vector()
        else:
            raise ValueError(" Input must be of type EulerAngles.")
        return UnitQuaternion(np.cos(eul[0] / 2.0), np.array([np.sin(eul[0] / 2.0), 0., 0.]))

    # Generate quaternion corresponding to a certain pitch angle (roll = yaw = 0)
    @staticmethod
    def from_pitch(eul: EulerAngles):  # e.g. EulerAngle.as_vector()
        if type(eul) == EulerAngles:
            eul = eul.as_vector()
        else:
            raise ValueError(" Input must be of type EulerAngles.")
        return UnitQuaternion(np.cos(eul[1] / 2.0), np.array([0.0, np.sin(eul[1] / 2.0), 0.0]))

    # Generate quaternion corresponding to a certain yaw angle (roll = pitch = 0)
    @staticmethod
    def from_yaw(eul: EulerAngles):  # e.g. EulerAngle.as_vector()
        if type(eul) == EulerAngles:
            eul = eul.as_vector()
        else:
            raise ValueError(" Input must be of type EulerAngles.")
        return UnitQuaternion(np.cos(eul[2] / 2.0), np.array([0.0, 0.0, np.sin(eul[2] / 2.0)]))

    # Generate quaternion from EulerAngles
    @staticmethod
    def from_euler(arg):  # EulerAngle.as_vector()
        if type(arg) is EulerAngles:
            eul: EulerAngles = arg
        elif type(arg) is np.array or type(arg) is list:
            eul = EulerAngles.from_vector(v=np.array(arg))
        else:
            raise ValueError(" Input must be of type EulerAngles or a 3D array or list.")

        return UnitQuaternion.from_yaw(eul) * UnitQuaternion.from_pitch(eul) * UnitQuaternion.from_roll(eul)

    @staticmethod
    def from_rotmat(R: Union[RotationMatrix, np.ndarray]):
        """ Generate Quaternion from RotationMatrix() """

        if type(R) == RotationMatrix:
            R = R.as_matrix()
        elif type(R) == np.ndarray:
            pass
        else:
            raise ValueError(" Input must be of type RotationMatrix.")

        angle = np.arccos((np.trace(R) - 1) / 2.0)
        real_part = np.cos(angle / 2.0)
        return UnitQuaternion(real_part, vex(asymproj(R)) / (2.0 * real_part))

    @staticmethod
    def from_euler_vector_deg(euler_vector):
        roll = euler_vector[0] * np.pi / 180
        pitch = euler_vector[1] * np.pi / 180
        yaw = euler_vector[2] * np.pi / 180

        q_yaw = UnitQuaternion(np.cos(yaw / 2.0), np.array([0.0, 0.0, np.sin(yaw / 2.0)]))
        q_pitch = UnitQuaternion(np.cos(pitch / 2.0), np.array([0.0, np.sin(pitch / 2.0), 0.0]))
        q_roll = UnitQuaternion(np.cos(roll / 2.0), np.array([np.sin(roll / 2.0), 0., 0.]))

        return q_yaw * q_pitch * q_roll

    @staticmethod
    def from_euler_angles_deg(roll=0, pitch=0, yaw=0):
        roll = roll * np.pi / 180
        pitch = pitch * np.pi / 180
        yaw = yaw * np.pi / 180

        q_yaw = UnitQuaternion(np.cos(yaw / 2.0), np.array([0.0, 0.0, np.sin(yaw / 2.0)]))
        q_pitch = UnitQuaternion(np.cos(pitch / 2.0), np.array([0.0, np.sin(pitch / 2.0), 0.0]))
        q_roll = UnitQuaternion(np.cos(roll / 2.0), np.array([np.sin(roll / 2.0), 0., 0.]))

        return q_yaw * q_pitch * q_roll

    @staticmethod
    def from_euler_angles_rad(roll=0, pitch=0, yaw=0):

        q_yaw = UnitQuaternion(np.cos(yaw / 2.0), np.array([0.0, 0.0, np.sin(yaw / 2.0)]))
        q_pitch = UnitQuaternion(np.cos(pitch / 2.0), np.array([0.0, np.sin(pitch / 2.0), 0.0]))
        q_roll = UnitQuaternion(np.cos(roll / 2.0), np.array([np.sin(roll / 2.0), 0., 0.]))

        return q_yaw * q_pitch * q_roll




