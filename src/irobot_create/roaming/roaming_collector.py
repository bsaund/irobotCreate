from irobot_create.robots.bradbot import Bradbot
import rospy
import numpy as np
from irobot_create.openinterface.constants import MODES

ROAM_SPEED = 100


def back_up_until_safe(bradbot, padding_time=0.5):
    bradbot.set_velocity_target(-ROAM_SPEED, -ROAM_SPEED)
    while bradbot.is_bumping() or bradbot.cliff_sensed():
        rospy.sleep(0.1)
    rospy.sleep(padding_time)
    print("Stopping")
    bradbot.set_velocity_target(0, 0)


def move_forward_until_bump(bradbot):
    """
    :param bradbot:
    ":type bradbot: Bradbot
    :return:
    """
    bradbot.set_velocity_target(ROAM_SPEED, ROAM_SPEED)
    # while not rospy.is_shutdown():
    #     print bradbot.is_bump()
    bradbot.time_in_stasis = 0.0
    while not bradbot.is_bumping() and not bradbot.cliff_sensed() \
            and not rospy.is_shutdown() \
            and bradbot.time_in_stasis < 1.0:
        rospy.sleep(0.01)
    print("Backing up")
    bradbot.set_velocity_target(-ROAM_SPEED, -ROAM_SPEED)
    back_up_until_safe(bradbot)

    rospy.sleep(0.1)


def random_turn(bradbot):
    direction = 1 - 2 * (np.random.random() > 0.8)
    s = direction * ROAM_SPEED
    bradbot.set_velocity_target(s, -s)
    turn_time = 100 * (0.5 + np.random.random()) / ROAM_SPEED
    rospy.sleep(turn_time)
    bradbot.set_velocity_target(0, 0)


def roam(bradbot):
    while not rospy.is_shutdown():
        if bradbot.oi_mode is MODES.PASSIVE:
            print("Failure! In passive mode. Probably cliff sensed")
            # return
            bradbot.set_katie_song()
            bradbot.oi_mode = MODES.SAFE
            back_up_until_safe(bradbot, padding_time=1.0)

        print("Moving Forward")
        move_forward_until_bump(bradbot)
        print("Turning")
        random_turn(bradbot)
