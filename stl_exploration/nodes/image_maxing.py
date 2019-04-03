#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import message_filters

image_pub = None
camera_info_pub = None
bridge = CvBridge()
max_meters = 7


def callback(image_msg, camera_info_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg, "32FC1")
    except CvBridgeError as e:
        print(e)

    height, width = cv_image.shape[0:2]
    for x in range(0, width):
        for y in range(0, height):
            if math.isnan(cv_image[y, x]):
                cv_image[y, x] = max_meters

    new_msg = bridge.cv2_to_imgmsg(cv_image, "32FC1")
    new_msg.header = image_msg.header

    image_pub.publish(new_msg)
    camera_info_pub.publish(camera_info_msg)


def image_maxing():
    global image_pub, camera_info_pub, camera_info
    image_pub = rospy.Publisher(
        "/virtual_camera/depth/image_raw", Image, queue_size=100)
    camera_info_pub = rospy.Publisher(
        "/virtual_camera/depth/camera_info", CameraInfo, queue_size=100)
    rospy.init_node("image_maxing", anonymous=True)

    image_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
    camera_info_sub = message_filters.Subscriber(
        "/camera/depth/camera_info", CameraInfo)
    ts = message_filters.TimeSynchronizer([image_sub, camera_info_sub], 10)
    ts.registerCallback(callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        image_maxing()
    except rospy.ROSInterruptException:
        pass
