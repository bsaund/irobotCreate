#!/usr/bin/python3
import jetson.inference
import jetson.utils
import rospy
from sensor_msgs.msg import Image, CompressedImage

# import IPython

net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
pub = None
# net = jetson.inference.detectNet("facenet-120", threshold=0.5)
# camera = jetson.utils.gstCamera(1280, 720, "0")
# camera = jetson.utils.gstCamera(1280, 720, "/dev/video1")
# display = jetson.utils.glDisplay()
#
# while display.IsOpen():
#     img, width, height = camera.CaptureRGBA()
#     detections = net.Detect(img, width, height)
#     # print(detections)
#     # IPython.embed()
#     # if len(detections) > 0:
#     #     print(detections[0].ClassID)
#
#     people = [d for d in detections if d.ClassID == 1]
#     if len(people) > 0:
#         c = people[0].Center[0]
#
#         if c < width / 3:
#             print("Left")
#         elif c > width * 2 / 3:
#             print("Right")
#         else:
#             print("I see you!")
#
#     display.RenderOnce(img, width, height)
#     display.SetTitle("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
def process_image(msg):
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("image_detection")
    pub = rospy.Publisher('bradbot_processed/compressed', CompressedImage)
    sub = rospy.Subscriber('bradbot_camera/compressed', CompressedImage, process_image)
    rospy.spin
