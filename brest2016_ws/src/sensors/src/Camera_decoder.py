#!/usr/bin/env python


# publie la video filmee par la camera


import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


def update_image(msg):
    global frame, bridge, flag
    frame = bridge.imgmsg_to_cv2(msg, "mono8")
    if not flag:
        flag = True

# Initiate node
rospy.init_node('video_decoding')

# Node rate
# rate = rospy.Rate(10)

# Bridge object
bridge = CvBridge()

# Subscriber to compressed image
sub = rospy.Subscriber('camera/image_compressed', Image, update_image)
# Publisher
# pub = rospy.Publisher('camera/image_uncompressed', Image, queue_size=1)

frame = np.ndarray(0)
flag = False
# Publishing the frame
while not rospy.is_shutdown():
    if flag:
        frame2 = cv2.imdecode(frame, 1)
        cv2.imshow('Image decompresse', frame2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # print type(frame2)
        # try:
        #     pub.publish(bridge.cv2_to_imgmsg(frame2, "bgr8"))
        # except CvBridgeError as e:
        #     print e
    # rate.sleep()
