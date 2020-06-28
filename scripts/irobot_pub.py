#! /usr/bin/env python
import rospy
import rospkg
from visualization_msgs.msg import Marker
import tf_conversions
import numpy as np

if __name__ == "__main__":
    rospy.init_node("bradbot_publisher")
    pub = rospy.Publisher("bradbot", Marker, queue_size=1)

    marker = Marker()
    marker.header.frame_id = "/bradbot"
    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD

    q = tf_conversions.transformations.quaternion_from_euler(np.pi/2, 0, np.pi/2)
    # q = [ 0.5, 0.5, 0.5, 0.5 ]
    marker.pose.orientation.w = q[3]
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.position.x = -.17
    marker.pose.position.y = -.17
    marker.pose.position.z = 0.0
    marker.scale.x = 0.001
    marker.scale.y = 0.001
    marker.scale.z = 0.001
    marker.color.a = 0.9
    marker.color.g = 1.0
    marker.color.r = .8
    marker.mesh_resource = "package://irobot_create/CAD/iRobot_iCreate.STL"

    while not rospy.is_shutdown():
        pub.publish(marker)
        rospy.Rate(10).sleep()
