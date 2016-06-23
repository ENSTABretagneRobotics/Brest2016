#!/usr/bin/env python


# publie la video filmee par la camera


import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Initiate node
rospy.init_node('video_capture')

# Node rate
rate = rospy.Rate(10)

# Camera path
cam_path = '/dev/video1'

# Bridge object
bridge = CvBridge()

# Publisher
pub = rospy.Publisher('camera/image_compressed', Image, queue_size=1)

# Capture object
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print 'Error while opening: ' + cam_path
    exit(0)
else:
    print 'Correctly opened ' + cam_path

# Publishing the frame
rval, frame = cap.read()
while rval:
    rval, frame = cap.read()
    r, frame2 = cv2.imencode('.jpeg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    try:
        pub.publish(bridge.cv2_to_imgmsg(frame2, "mono8"))
        # cv2.imdecode(frame2,1) a rajouter en sortie
    except CvBridgeError as e:
        print e
    rate.sleep()
