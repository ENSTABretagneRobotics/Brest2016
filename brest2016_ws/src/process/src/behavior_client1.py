#!/usr/bin/env  python
import rospy
from process.srv import behavior
from process.msg import BehaviorInfo

rospy.init_node('behavior_client')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# Behavior a envoyer
info0 = BehaviorInfo(behavior_id='000', type='limite',
                     xa=128, ya=99, xb=170, yb=104, s=-3, r=4)
info1 = BehaviorInfo(behavior_id='001', type='limite',
                     xa=170, ya=104, xb=173, yb=71, s=-3, r=4)
info2 = BehaviorInfo(behavior_id='002', type='limite',
                     xa=173, ya=71, xb=132, yb=63, s=-3, r=4)
info3 = BehaviorInfo(behavior_id='003', type='limite',
                     xa=132, ya=63, xb=128, yb=99, s=-3, r=4)

# Confirmation de la reception du behavior
rate = rospy.Rate(1)
for info in reversed([info0, info1, info2, info3]):
    confirmation = behavior_sender(info)
    print confirmation
    rate.sleep()
