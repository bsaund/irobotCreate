import time
import numpy as np


class PID:
    """
    Implemented a simple PID controller
    TODO: Currently only a simple P controller
    """

    def __init__(self, p=1, i=0, d=0):
        self.target = None
        self.p = p
        self.i = i
        self.d = d
        self.prev_update_time = time.time.now()

    def run(self):
        pass


class LowPassController:
    """
    Implementes a simple lowpass filter
    """

    def __init__(self, T=1, start_val=0):
        """
        :param T: charactaristic time in s
        """
        self.T = T
        self.current_value = start_val
        self.target = None
        self.prev_update_time = time.time()

    def set_target(self, target):
        self.current_value = self.get_current_value()

        self.prev_update_time = time.time()
        self.target = target

    def get_current_value(self):
        if self.target is None:
            return self.current_value
        dt = time.time() - self.prev_update_time
        a = np.exp(-dt / self.T)
        return a * self.current_value + (1 - a) * self.target
        # return self.current_value


class RampController:
    """
    Implements a simple ramp controller
    """

    def __init__(self, max_accel=1, start_val=0):
        self.accel = max_accel
        self.current_value = start_val
        self.target = None
        self.last_update_time = time.time()

    def set_target(self, target):
        self.get_current_value()
        self.target = target

    def get_current_value(self):
        if self.target is None:
            return self.current_value
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        dx = self.target - self.current_value
        if np.abs(dx) < dt * self.accel:
            self.current_value = self.target
            return self.current_value

        self.current_value += dt * self.accel * np.sign(dx)
        return self.current_value

