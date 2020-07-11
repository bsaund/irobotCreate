from irobot_create.robots.bradbot import Bradbot
import rospy
import numpy as np

ROAM_SPEED = 100


def move_forward_until_bump(bradbot):
    """
    :param bradbot:
    ":type bradbot: Bradbot
    :return:
    """
    bradbot.set_velocity_target(ROAM_SPEED, ROAM_SPEED)
    # while not rospy.is_shutdown():
    #     print bradbot.is_bump()
    while not bradbot.is_bumping() and not rospy.is_shutdown():
        # print bradbot.is_bumping()
        rospy.sleep(0.01)
    print("Backing up")
    bradbot.set_velocity_target(-ROAM_SPEED, -ROAM_SPEED)
    while bradbot.is_bumping():
        rospy.sleep(0.1)
    rospy.sleep(1)
    print("Stopping")
    bradbot.set_velocity_target(0, 0)
    rospy.sleep(0.1)


def random_turn(bradbot):
    direction = 1 - 2 * (np.random.random() > 0.5)
    s = direction * ROAM_SPEED
    bradbot.set_velocity_target(s, -s)
    turn_time = 100 * (1 + np.random.random()) / ROAM_SPEED
    rospy.sleep(turn_time)
    bradbot.set_velocity_target(0, 0)


def roam(bradbot):
    while not rospy.is_shutdown():
        print("Moving Forward")
        move_forward_until_bump(bradbot)
        print("Turning")
        random_turn(bradbot)
