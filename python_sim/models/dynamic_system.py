import numpy as np
from abc import ABCMeta


class DiscreteDynamicSystem(metaclass=ABCMeta):

    """ Dynamic System class to simulate nonlinear systems. Uses rk4 to increment the system.
    Child classes must implement _dynamics function for the nonlinear system: x_dot = f(x, u, t) """

    def __init__(self, x_init: np.array):
        self.state = x_init
        self._state_dot = np.zeros([len(self.state)])

    def _dynamics(self, x, u, t):
        """ x_dot = f(x, u, t) """
        pass

    def rk4(self, u_i, t_i, dt):

        x_i = self.state

        d1 = dt * self._dynamics(x_i, u_i, t_i)
        d2 = dt * self._dynamics(x_i + d1 / 2, u_i, t_i + dt / 2)
        d3 = dt * self._dynamics(x_i + d2 / 2, u_i, t_i + dt / 2)
        d4 = dt * self._dynamics(x_i + d3, u_i, t_i + dt)

        increment = (d1 + 2 * d2 + 2 * d3 + d4) / 6

        x_i_1 = x_i + increment
        t_i_1 = t_i + dt

        self.state = x_i_1
        self._state_dot = increment / dt

        return x_i_1, t_i_1
