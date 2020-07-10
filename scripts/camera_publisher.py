#!/usr/bin/env python

from irobot_create.vision import jetson_camera
import rospy
from sensor_msgs.msg import Image, CompressedImage

if __name__ == "__main__":
    rospy.init_node("camera_publisher")

    camera = jetson_camera.open_camera()

    pub = rospy.Publisher("/bradbot_camera", CompressedImage, queue_size=1)

    while not rospy.is_shutdown():
        pub.publish(jetson_camera.get_image_msg(camera))
        rospy.Rate(10).sleep()
