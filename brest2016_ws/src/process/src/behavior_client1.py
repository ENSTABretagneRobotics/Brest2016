#!/usr/bin/env  python
import rospy
from process.srv import behavior
from process.msg import BehaviorInfo

rospy.init_node('behavior_client')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# Behavior a envoyer
info0 = BehaviorInfo(behavior_id='000', type='limite',
                     xa=10, ya=25, xb=74, yb=34, s=-1, r=10)
info1 = BehaviorInfo(behavior_id='001', type='limite',
                     xa=74, ya=34, xb=62, yb=132, s=-1, r=10)
info2 = BehaviorInfo(behavior_id='002', type='limite',
                     xa=62, ya=132, xb=-1, yb=125, s=-1, r=10)
info3 = BehaviorInfo(behavior_id='003', type='limite',
                     xa=-1, ya=125, xb=10, yb=25, s=-1, r=10)

# Confirmation de la reception du behavior
rate = rospy.Rate(1)
for info in reversed([info0, info1, info2, info3]):
    confirmation = behavior_sender(info)
    print confirmation
    rate.sleep()
