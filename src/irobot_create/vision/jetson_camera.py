import jetson.utils
import rospy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2


def open_camera():
    camera = jetson.utils.gstCamera(1280, 720, "/dev/video1")
    return camera


def get_image_msg(camera):
    img_gpu, width, height = camera.CaptureRGBA(zeroCopy=True)
    return gpu_img_to_img_msg(img_gpu, width, height)
    # m = CompressedImage()
    # # m.width = width
    # # m.height = height
    # jetson.utils.cudaDeviceSynchronize()
    # img_rgba = jetson.utils.cudaToNumpy(img_gpu, width, height, 4).astype(np.uint8)

    # img = cv2.cvtColor(img_rgba[:,:,0:3], cv2.COLOR_BGR2RGB)
    # m.data = np.array(cv2.imencode(".png", img)[1]).tostring()
    # m.format = "png"

    # return m


def gpu_img_to_img_msg(img_gpu, width, height):
    m = CompressedImage()
    jetson.utils.cudaDeviceSynchronize()
    img_rgba = jetson.utils.cudaToNumpy(img_gpu, width, height, 4).astype(np.uint8)

    img = cv2.cvtColor(img_rgba[:,:,0:3], cv2.COLOR_BGR2RGB)
    m.data = np.array(cv2.imencode(".png", img)[1]).tostring()
    m.format = "png"

    return m

