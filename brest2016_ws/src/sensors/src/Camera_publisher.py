#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import glob

# Initiate node
rospy.init_node('video_capture')

# Node rate
rate = rospy.Rate(10)

# Camera path
cam_path = glob.glob('/dev/video*')[0]
index = int(cam_path[-1])
print index

# Bridge object
bridge = CvBridge()

# Publisher
pub = rospy.Publisher('camera/image_raw', Image, queue_size=1)

# Capture object
cap = cv2.VideoCapture(index)

if not cap.isOpened():
    print 'Error while opening: ' + cam_path
    exit(0)
else:
    print 'Correctly opened ' + cam_path

# Publishing the frame
rval, frame = cap.read()
while rval:
    rval, frame = cap.read()
    try:
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    except CvBridgeError as e:
        print e
    except KeyboardInterrupt as e:
        cap.release()
        print 'camera released'
    rate.sleep()
