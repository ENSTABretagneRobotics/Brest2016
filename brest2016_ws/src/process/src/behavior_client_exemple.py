#!/usr/bin/env  python


# envoie un behavior au serveur


import rospy
from process.srv import behavior
from process.msg import BehaviorInfo


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print 'parameter [%s] not defined.' % name
        print 'Defaulting to', default
        return default


rospy.init_node('behavior_client')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)


x = fetch_param('~x', 20)
y = fetch_param('~y', 20)
x2 = fetch_param('~xb', 20)
y2 = fetch_param('~yb', 20)
behav_type = fetch_param('~behav_type', 'waypoint')
force = fetch_param('~force', 1)
radius = fetch_param('~radius', 3)
slowr = fetch_param('~slowr', 3)
b_id = str(fetch_param('~b_id', '000'))
e_range = fetch_param('~e_range', 5)

mode = fetch_param('~mode', 'update')
# Behavior a envoyer
info = BehaviorInfo(behavior_id=b_id, f_type=behav_type,
                    xa=x, ya=y,
                    xb=x2, yb=y2,
                    K=force, R=radius,
                    security='LOW', slowing_R=slowr,
                    slowing_K=1,
                    effect_range=e_range)
confirmation = behavior_sender(info, mode)
rospy.loginfo(confirmation)
