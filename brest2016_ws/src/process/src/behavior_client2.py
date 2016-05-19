#!/usr/bin/env  python
import rospy
from process.srv import behavior
from process.msg import BehaviorInfo

rospy.init_node('behavior_client')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# Behavior a envoyer
info0 = BehaviorInfo(behavior_id='000', type='waypoint',
                     xa=152, ya=82, xb=0, yb=0, s=-1, r=10)

# Confirmation de la reception du behavior
rate = rospy.Rate(1)
confirmation = behavior_sender(info0)
print confirmation
rate.sleep()
