import jetson.utils
import rospy
from sensor_msgs.msg import Image
import numpy as np
import IPython


def open_camera():
    camera = jetson.utils.gstCamera(1280, 720, "/dev/video1")
    return camera


def get_image_msg(camera):
    img, width, height = camera.CaptureRGBA(zeroCopy=True)
    m = Image()
    m.width = width
    m.height = height
    m.data = jetson.utils.cudaToNumpy(img, width, height, 4).astype(np.uint8).flatten().tolist()
    m.encoding="rgba8"
    # IPython.embed()
    return m
