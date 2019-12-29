#!/usr/bin/env python

"""
Dead reckoning for the irobot
"""

import numpy as np

TICKS_PER_RAD = 275.0
TICKS_PER_METER = 2472.0


class Cartesian:
    def __init__(self, left_enc, right_enc):
        self.x = 0
        self.y = 0
        self.theta = 0

        self.left_enc = left_enc
        self.right_enc = right_enc


    def update(self, left_enc, right_enc):
        d_left = self.loop(left_enc - self.left_enc)
        d_right = self.loop(right_enc - self.right_enc)

        self.left_enc = left_enc
        self.right_enc = right_enc


        d_forward = (d_left + d_right)/ (2*TICKS_PER_METER)
        d_theta = (d_right - d_left) / (2*TICKS_PER_RAD)

        self.theta += d_theta

        self.x += d_forward * np.cos(self.theta)
        self.y += d_forward * np.sin(self.theta)

        

    """
    handles the 0 - 65535 loop
    """
    def loop(self, d_enc, loop_limit = 65536):
        if d_enc <  -1 * loop_limit / 2:
            return d_enc + loop_limit
        if d_enc > loop_limit / 2:
            return d_enc - loop_limit
        
        return d_enc
