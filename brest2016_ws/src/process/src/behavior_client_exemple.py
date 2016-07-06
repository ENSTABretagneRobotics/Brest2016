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
# Behavior a envoyer
info = BehaviorInfo(behavior_id='002', f_type='waypoint',
                    xa=x, ya=y,
                    xb=0, yb=0,
                    K=1, R=3,
                    security='LOW', slowing_R=1,
                    slowing_K=1,
                    effect_range=4)
confirmation = behavior_sender(info, 'update')
