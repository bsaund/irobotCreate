import jetson.utils
import rospy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import IPython
import cv2


def open_camera():
    camera = jetson.utils.gstCamera(1280, 720, "/dev/video1")
    return camera


def get_image_msg(camera):
    img_gpu, width, height = camera.CaptureRGBA(zeroCopy=True)
    m = Image()
    m.width = width
    m.height = height
    img = jetson.utils.cudaToNumpy(img_gpu, width, height, 4).astype(np.uint8)
    # m.data = img.flatten().tolist()
    m.data = np.array(cv2.imencode(".jpg", img)[1]).tostring()
    m.format = "jpeg"
    # m.encoding="rgba8"
    # IPython.embed()
    return m
