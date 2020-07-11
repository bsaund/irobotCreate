#!/usr/bin/env python


import rospy
from irobot_create.robots.bradbot import Bradbot
from irobot_create.roaming import roaming_collector
from irobot_create.openinterface.constants import MODES

if __name__ == "__main__":
    rospy.init_node("roaming_collector_node")
    bradbot = Bradbot("/dev/ttyUSB0", remote=True)
    bradbot.oi_mode = MODES.SAFE
    bradbot.start_control_loop_thread()

    # while not rospy.is_shutdown():
    #     all_sensors = bradbot.irobot_data
    #     bumps = all_sensors.bumps_and_wheel_drops
    #     print bumps.bump_left
    #     rospy.sleep(0.1)

    # roaming_collector.move_forward_until_bump(bradbot)

    # for _ in range(10):
    #     roaming_collector.random_turn(bradbot)
    #     rospy.sleep(0.5)
    roaming_collector.roam(bradbot)

    bradbot.shutdown()
