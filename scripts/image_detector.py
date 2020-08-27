#!/usr/bin/python3
import jetson.inference
import jetson.utils
import rospy
from sensor_msgs.msg import Image, CompressedImage

#!/usr/bin/env python

from irobot_create.vision import jetson_camera
import rospy
from sensor_msgs.msg import Image, CompressedImage

if __name__ == "__main__":
    rospy.init_node("image_detection_publisher")

    camera = jetson_camera.open_camera()
    net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

    pub = rospy.Publisher("/bradbot_vision_processed/compressed", CompressedImage, queue_size=1)
    pub_raw = rospy.Publisher("/bradbot_camera/compressed", CompressedImage, queue_size=1)

    while not rospy.is_shutdown():
        img, width, height = camera.CaptureRGBA(zeroCopy=True)

        # pub_raw.publish(jetson_camera.gpu_img_to_img_msg(img, width, height))
        
        detections = net.Detect(img, width, height)
        
        # pub.publish(jetson_camera.get_image_msg(camera))
        pub.publish(jetson_camera.gpu_img_to_img_msg(img, width, height))
        # rospy.Rate(10).sleep()
