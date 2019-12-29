#!/usr/bin/env python

from irobot.robots import create2
import cartesian


class Bradbot(create2.Create2):
    def __init__(self, serial):
        super(Bradbot, self).__init__(serial)
        
        self.irobot_data = None
        self.read_irobot_data()
        self.pos = cartesian.Cartesian(self.irobot_data.left_encoder_counts, self.irobot_data.right_encoder_counts)
        

    def read_irobot_data(self):
        self.irobot_data = self.sensor_group100

    def update_from_sensors(self):
        self.read_irobot_data()
        self.pos.update(self.irobot_data.left_encoder_counts, self.irobot_data.right_encoder_counts)
