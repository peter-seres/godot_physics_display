import numpy as np
from .dynamic_system import DiscreteDynamicSystem
from kinematics import UnitQuaternion


class UAV(DiscreteDynamicSystem):
    """
    Drone with 6 degrees of freedom: Body coordinate system: X forward Y right Z down

        States x :

            0-1-2: position inertial (N E D)

            3-4-5: velocity inertial (N E D)

            6-7-8-9: unit quaternion (q_w q_i q_j q_k)

            10-11-12: angular rates (p q r)

        Inputs u : (wrench)

            0-1-2: body forces (X, Y, Z)

            4-5-6: body moments (L, M, N)

        Parameters:

            body mass

            inertia matrix
    """

    def __init__(self, x_0=None, mass=1.0, inertia_matrix=None):

        # Parameters:
        self.m = mass                   # vehicle mass                                  [kg]

        if inertia_matrix is None:
            self.I_v = np.eye(3)
        else:
            self.I_v = inertia_matrix   # vehicle inertia matrix                        [kgm2]

        # Initial conditions:
        if x_0 is None:
            x_0 = np.zeros(shape=[13])
            x_0[6:10] = UnitQuaternion.from_euler_angles_deg(roll=0, pitch=0, yaw=0).as_vector()

        super().__init__(x_init=x_0)
        self.I_v_inverse = np.linalg.inv(self.I_v)

    @property
    def position(self):
        """ Returns NED position vector. """

        return self.state[0:3]

    @property
    def position_tuple(self):
        """ Returns NED position tuple. """

        return self.state[0], self.state[1], self.state[2]

    @property
    def velocity(self):
        """ Returns NED inertial velocity vector. """

        return self.state[3:6]

    @property
    def orientation(self):
        """ Returns orientation unit quaternion. """

        return UnitQuaternion.from_wxyz(self.state[6:10])

    @property
    def omega(self):
        """ Returns angular velocity vector. """

        return self.state[10:]

    @property
    def heading(self):
        """ Heading angle relative to North in radians."""

        return self.orientation_euler_rad[2]

    def get_frame_position(self, position_on_body):
        """ Returns the NED intertial position of any location defined in the body frame. """

        return self.position + self.orientation * position_on_body

    def get_frame_velocity(self, position_on_body):
        """ Returns the NED intertial velocity of any location defined in the rotating body frame. """

        velocity_at_position = self.velocity + self.orientation * (np.cross(self.omega, position_on_body))

        return velocity_at_position

    def get_acceleration_inertial(self):
        """ Returns the linear acceleration vector in the NED frame. """
        return self._state_dot[3:6]

    def get_acceleration_body(self):
        """ Returns the linear acceleration vector in the body frame. """

        return self.orientation.inverse() * self._state_dot[3:6]

    def get_angular_acceleration(self):
        """ Returns the angular acceleration vector. """

        return self._state_dot[10:13]

    def get_wrench(self):
        """ Returns the combined 6 element vector of the linear accelerations and rotational accelerations. """

        return np.concatenate([self.get_acceleration_body(), self.get_angular_acceleration()])

    def get_airspeed_body(self, wind_field_inertial):
        """ Returns the body frame airspeed at the center of gravity. """

        airspeed_inertial = self.velocity - wind_field_inertial
        airspeed_body = self.orientation.inverse() * airspeed_inertial

        return airspeed_body

    def get_airspeed_at_position(self, position_on_body, wind_field_inertial):
        """ Returns the body frame airspeed at a position on the body accounted for the rotating reference frame. """

        airspeed = self.get_airspeed_body(wind_field_inertial)
        airspeed_at_position = airspeed + self.orientation * np.cross(self.omega, position_on_body)

        return airspeed_at_position

    @property
    def orientation_euler_rad(self):
        """ Return euler angle representation of orientation. """
        eul_rad = self.orientation.as_euler_vector()
        if eul_rad[2] < 0:
            eul_rad[2] += 2 * np.pi
        return eul_rad

    @property
    def orientation_euler_deg(self):
        """ Return euler angle representation of orientation. """
        eul_deg = self.orientation.as_euler_vector() * 180 / np.pi
        if eul_deg[2] < 0:
            eul_deg[2] += 360.0
        return eul_deg

    def __repr__(self):
        return f"    Position NED: {self.position} \n " \
               f"    Velocity NED: {self.velocity} \n " \
               f"    Attitude: {self.orientation_euler_deg} \n " \
               f"    Omega: {self.omega}"

    def _dynamics(self, x, u, t):
        """
        States x :

            0-1-2: position inertial (N E D)

            3-4-5: velocity inertial (N E D)

            6-7-8-9: unit quaternion (q0 q1 q2 q3)

            10-11-12: angular rates (p q r)

        Inputs u : (wrench)

            0-1-2: body forces (X, Y, Z)

            4-5-6: body moments (L, M, N)

        """

        # Initialize state perturbation:
        x_dot = np.zeros(shape=[13])

        # Orientation
        q = UnitQuaternion.from_wxyz(x[6:10])

        # Inputs:
        F = u[0:3]  # Forces
        M = u[3:6]  # Moments

        # Linear kinematics:
        p_dot = x[3:6]

        # Linear dynamics:
        v_dot = q * F / self.m

        # Rotational kinematics:
        omega = x[10:]
        q_dot = q.q_dot(omega)

        # Rotational dynamics:
        omega_dot = self.I_v_inverse @ (M - np.cross(omega, self.I_v @ omega))

        # Assign to state vector:
        x_dot[0:3] = p_dot
        x_dot[3:6] = v_dot
        x_dot[6:10] = q_dot
        x_dot[10:13] = omega_dot

        return x_dot
