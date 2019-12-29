#!/usr/bin/env python

from irobot.robots import create2
import cartesian
import numpy as np
import time





class Bradbot(create2.Create2):
    def __init__(self, serial):
        super(Bradbot, self).__init__(serial)
        
        self.irobot_data = None
        self.read_irobot_data()
        self.pos = cartesian.Cartesian(self.irobot_data.left_encoder_counts, self.irobot_data.right_encoder_counts)

        self.x_target = None
        self.y_target = None
        self.err_limit = None
        self.vel = None
        self.moving = False
        

    def read_irobot_data(self):
        self.irobot_data = self.sensor_group100

    def update_from_sensors(self):
        self.read_irobot_data()
        self.pos.update(self.irobot_data.left_encoder_counts, self.irobot_data.right_encoder_counts)

    def is_bump(self):
        bumps = self.irobot_data.bumps_and_wheel_drops
        if bumps.bump_left or bumps.bump_right:
            return True
        lt = self.irobot_data.light_bumper
        if lt.left or lt.front_left or lt.center_left or lt.center_right or lt.front_right or lt.right:
            return True
        return False

    """
    Main control loop. Call this to keep this periodically to keep the robot sensing and moving
    """
    def control_loop(self):
        self.update_from_sensors()
        self.handle_move()

    def handle_move(self):
        if not self.moving:
            return

        if np.linalg.norm([self.pos.x - self.x_target, self.pos.y - self.y_target]) <= self.err_limit:
            print("Reached! Stopping")
            self.moving = False

        if self.is_bump():
            print("Bumped! Stopping")
            self.moving = False
        
        if not self.moving:
            self.drive_direct(0,0)
            return


        vr, vl = self.simple_control(self.x_target, self.y_target, self.vel)    
        self.drive_direct(vr, vl)        

    """
    Commands the robot to move to the target, returning true if reached within error limit.
    Blocks
    """
    def go_to(self, x_target, y_target, vel=200, err_limit = 0.05):
        self.moving = True
        self.x_target = x_target
        self.y_target = y_target
        self.vel = vel
        self.err_limit = err_limit


    """
    returns (vr, vl) wheel velocities using a simple controller
    """
    def simple_control(self, x_target, y_target, vel):
        th_limit = 0.1 #rad
        
        """reset the robot origin to 0"""
        x_target -= self.pos.x
        y_target -= self.pos.y
        th = self.pos.theta

        x_target_tmp = x_target * np.cos(th) - y_target * np.sin(th)
        y_target = x_target * np.sin(th) + y_target * np.cos(th)
        x_target = x_target_tmp
    

        th_target = np.arctan2(y_target, x_target)
        print("th: %.2f, x: %.2f, y: %.2f" % (th_target, x_target, y_target))

        vr = min(vel, vel * np.abs(th_target)*3) * np.sign(th_target)
        vl = -vr
        
        if np.abs(th_target) > th_limit:
            return vr, vl

        vr *= 2
        vl *= 2
        vr += vel
        vl += vel

        scale = vel / max(np.abs(vr), np.abs(vl))
        vr *= scale
        vl *= scale

        return vr, vl
