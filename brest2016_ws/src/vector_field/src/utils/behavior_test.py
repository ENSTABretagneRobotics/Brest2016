# ! todo: A CORRIGER !
import numpy as np
from behavior import Behavior
import matplotlib.pyplot as plt
from behaviorManager import BehaviorManager
##########################################################################
# Behavior INFO     TODO: replace with behavior info message
##########################################################################


class Behavior_info():
    def __init__(self, f_type, behavior_id, xa, ya, xb=0, yb=0, K=1, R=0,
                 slowing_R=0, slowing_K=0, security='HIGH', effect_range=10):
        self.f_type = f_type
        self.behavior_id = behavior_id
        self.R = R
        self.K = K
        self.xa = xa
        self.ya = ya
        self.xb = xb
        self.yb = yb
        self.R = R
        self.K = K
        self.slowing_R = slowing_R
        self.slowing_K = slowing_K
        self.security = security
        self.effect_range = effect_range
##########################################################################
# TESTs
##########################################################################


def behaviorSimple():
    X, Y = np.mgrid[-100:100:40j, -100:100:40j]
    # Behavior 3 - patrol_circle
    # Behavior 1 - waypoint
    info1 = Behavior_info(f_type='waypoint', behavior_id='001', xa=40, ya=40,
                          xb=0, yb=0, K=1, R=0, slowing_R=0, slowing_K=0,
                          security='HIGH', effect_range=10)
    b1 = Behavior(info1)
    b1.projection = True
    U1, V1 = b1.get_field(X, Y, wind=78, theta=100)
    print b1.get_field(0, 0, wind=45, theta=100)
    plt.figure(1)
    plt.quiver(X, Y, U1, V1)
    plt.show()


def main_behavior():
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]

    # ##########################################
    # Behavior 0
    info0 = Behavior_info(f_type='constant', behavior_id='000', xa=0, ya=1)
    b0 = Behavior(info0)
    # U0, V0 = b0.get_field(X, Y)
    # plt.figure(0)
    # plt.quiver(X, Y, U0, V0)

    # ##########################################
    # Behavior 1 - waypoint
    info1 = Behavior_info(f_type='waypoint', behavior_id='001', xa=5, ya=10,
                          xb=0, yb=0, K=1, R=0, slowing_R=0, slowing_K=0,
                          security='HIGH', effect_range=10)
    b1 = Behavior(info1)
    # U1, V1 = b1.get_field(X, Y)
    # plt.figure(1)
    # plt.quiver(X, Y, U1, V1)

    # ##########################################
    # Behavior 2 - limite
    info2 = Behavior_info(f_type='limite', behavior_id='002', xa=-5, ya=0,
                          xb=5, yb=0, K=3, R=5, slowing_R=1, slowing_K=5,
                          security='MEDIUM', effect_range=10)
    b2 = Behavior(info2)
    # U2, V2 = b2.get_field(X, Y)
    # plt.figure(2)
    # plt.quiver(X, Y, U2, V2)

    # ##########################################
    # Behavior 3 - patrol_circle
    info3 = Behavior_info(f_type='patrol_circle', behavior_id='003',
                          xa=0, ya=0, K=3, R=5)
    b3 = Behavior(info3)
    # b3.projection = True
    # U3, V3 = b3.get_field(X, Y)
    # plt.figure('patrol_circle')
    # plt.quiver(X, Y, U3, V3)

    # ##########################################
    # Behavior 4 - ligne
    info4 = Behavior_info(f_type='ligne', behavior_id='004',
                          xa=0, ya=0, xb=10, yb=0, K=4, R=5, effect_range=50)
    b4 = Behavior(info4)
    # U4, V4 = b4.get_field(X, Y)
    # plt.figure(4)
    # plt.quiver(X, Y, U4, V4)

    # ##########################################
    # Behavior Total
    b = b1 + b2
    b.projection = True
    U, V = b.get_field(X, Y, wind=90, theta=100, normalize=True)
    print b.get_field(0, 0, wind=90, theta=100)
    plt.figure('Total')
    plt.quiver(X, Y, U, V)

    plt.show()


def main_manager():
    X, Y = np.mgrid[-100:100:40j, -100:100:40j]
    manager = BehaviorManager(sailboat=True)

    info1 = Behavior_info(f_type='waypoint', behavior_id='001', xa=40, ya=40,
                          xb=100, yb=0,
                          K=1, R=20, slowing_R=50, slowing_K=5,
                          security='LOW', effect_range=100)
    b1 = Behavior(info1)
    # Behavior 2 - limite
    info2 = Behavior_info(f_type='obst_point2', behavior_id='002', xa=20, ya=20,
                          xb=5, yb=5, K=3, R=10, slowing_R=10, slowing_K=5,
                          security='LOW', effect_range=10)
    b2 = Behavior(info2)
    # Behavior 3 - patrol_circle
    info3 = Behavior_info(f_type='patrol_circle', behavior_id='003',
                          xa=0, ya=0, K=3, R=10, slowing_R=30)
    b3 = Behavior(info3)

    print manager.champ_total.get_field(0, 0, 45, 100)
    print manager.champ_total.behavior_function
    manager.handle_behavior(b1, 'update')
    # manager.add_behavior(b2)
    # manager.handle_behavior(b2, 'update')
    # manager.sailboat = True
    # manager.handle_behavior(b3, 'update')

    res = manager.champ_total.get_field(X, Y, 45, 100, normalize=True)
    print manager.champ_total.get_field(0, 0, 45, 100)

    U, V = res[0], res[1]
    plt.quiver(X, Y, U, V)

    # manager.handle_behavior(b3, 'remove')
    # plt.figure(33)
    # res = manager.champ_total.get_field(X, Y, 45, 100, normalize=True)
    # U, V = res[0], res[1]
    # plt.quiver(X, Y, U, V)
    plt.show()


def test3():
    X, Y = np.mgrid[-100:100:40j, -100:100:40j]
    manager = BehaviorManager(sailboat=True)
    # Behavior 3 - patrol_circle
    info3 = Behavior_info(f_type='patrol_circle', behavior_id='003',
                          xa=0, ya=0, K=3, R=10, slowing_R=30)
    b3 = Behavior(info3)
    manager.handle_behavior(b3, 'remove')
    res = manager.champ_total.get_field(X, Y, 45, 100, normalize=True)
    U, V = res[0], res[1]
    plt.quiver(X, Y, U, V)
    plt.show()


if __name__ == '__main__':
    # behaviorSimple()
    # main_behavior()
    main_manager()
    # test3()
