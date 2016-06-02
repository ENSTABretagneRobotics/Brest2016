#!/usr/bin/env  python
import rospy
from process.srv import behavior
from process.msg import BehaviorInfo

rospy.init_node('behavior_client')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

# Behavior a envoyer
# info0 = BehaviorInfo(behavior_id='000', type='waypoint',
#                      xa=60, ya=-30, xb=0, yb=0, s=-0.5, r=1)
# info1 = BehaviorInfo(behavior_id='001', type='limite',
#                      xa=-30, ya=2.5, xb=30, yb=2.5, s=-0, r=10)
# info2 = BehaviorInfo(behavior_id='002', type='limite',
#                      xa=30, ya=2.5, xb=30, yb=-57.5, s=-2, r=10)

info0 = BehaviorInfo(f_type='waypoint', behavior_id='000', xa=60, ya=-30,
                     xb=0, yb=0, K=0.5, R=1, slowing_R=1, slowing_K=5,
                     security='HIGH')
info1 = BehaviorInfo(f_type='limite', behavior_id='001', xa=-30, ya=2.5,
                     xb=30, yb=2.5, K=2, R=10, slowing_R=1, slowing_K=5,
                     security='LOW', effect_range=10)
info2 = BehaviorInfo(f_type='limite', behavior_id='002', xa=30, ya=2.5,
                     xb=30, yb=-57.5, K=2, R=10, slowing_R=1, slowing_K=5,
                     security='LOW', effect_range=10)

# Confirmation de la reception du behavior
rate = rospy.Rate(1)
for info in reversed([info0, info1, info2]):
    confirmation = behavior_sender(info)
    print confirmation
    rate.sleep()
