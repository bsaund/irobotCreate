#!/usr/bin/python3
import jetson.inference
import jetson.utils
from irobot_create.vision import jetson_camera

import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

import json
import IPython

def detections_to_dict(detections):
    keys = ['Bottom', 'Top', 'ClassID', 'Left', 'Right', 'Confidence']
    return [{k: getattr(d, k) for k in keys} for d in detections]

if __name__ == "__main__":
    rospy.init_node("image_detection_publisher")

    camera = jetson_camera.open_camera()
    net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

    pub = rospy.Publisher("/bradbot_vision_processed/compressed", CompressedImage, queue_size=1)
    pub_raw = rospy.Publisher("/bradbot_camera/compressed", CompressedImage, queue_size=1)
    pub_detections = rospy.Publisher("detections", String, queue_size=1)

    while not rospy.is_shutdown():
        img, width, height = camera.CaptureRGBA(zeroCopy=True)

        # pub_raw.publish(jetson_camera.gpu_img_to_img_msg(img, width, height))
        
        detections = net.Detect(img, width, height)
        # IPython.embed()
        
        # pub.publish(jetson_camera.get_image_msg(camera))
        pub.publish(jetson_camera.gpu_img_to_img_msg(img, width, height))
        pub_detections.publish(json.dumps(detections_to_dict(detections)))
        # rospy.Rate(10).sleep()
