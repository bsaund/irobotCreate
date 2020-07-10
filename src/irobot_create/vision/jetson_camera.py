import jetson.utils
import rospy
from sensor_msgs.msg import Image


def open_camera():
    camera = jetson.utils.gstCamera(1280, 720, "/dev/video1")
    return camera


def get_image_msg(camera):
    img, width, height = camera.CaptureRGBA()
    m = Image()
    m.width = width
    m.height = height
    m.data = img
    return m
