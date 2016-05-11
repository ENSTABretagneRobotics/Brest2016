#!/usr/bin/env  python
import rospy
from process.srv import behavior, behaviorResponse
from Behavior import Behavior


def add_behavior(request):
    print type(request)
    print request.data
    # b = Behavior(request)
    return behaviorResponse('oui')
rospy.init_node('service_server')
service = rospy.Service('behavior_manager', behavior, add_behavior)
rospy.spin()
    