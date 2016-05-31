#!/usr/bin/env  python
import rospy
from process.srv import behavior
from process.msg import BehaviorInfo

rospy.init_node('behavior_client')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)


info0 = BehaviorInfo(f_type='waypoint', behavior_id='000', xa=60, ya=60,
                          xb=0, yb=0, K=0.5, R=1, slowing_R=1, slowing_K=5,
                          security='HIGH')
info1 = BehaviorInfo(f_type='obst_point', behavior_id='001', xa=0, ya=0,
                          xb=0, yb=0, K=2, R=15, slowing_R=1, slowing_K=5,
                          security='LOW', effect_range=10)
# info2 = BehaviorInfo(f_type='limite', behavior_id='002', xa=30, ya=2.5,
#                           xb=30, yb=-57.5, K=2, R=10, slowing_R=1, slowing_K=5,
#                           security='LOW', effect_range=10)

# Confirmation de la reception du behavior
rate = rospy.Rate(1)
for info in reversed([info0, info1]):
    confirmation = behavior_sender(info)
    print confirmation
    rate.sleep()

