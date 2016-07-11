# defini les differents types de behavior
# et les informations qu il doivent contenir

# -*- coding: utf-8 -*-
# import champs_numpy as cn
import vectorFieldLib as vfl
import numpy as np
import matplotlib.pyplot as plt

##########################################################################
# Behavior
##########################################################################


class Behavior(object):
    """A behavior generates a vector field it can be added to others"""

    corr = {'patrol_circle': vfl.patrouille_circulaire,
            'ligne': vfl.ligne,
            'limite': vfl.limite,
            'waypoint': vfl.waypoint,
            'constant': vfl.champ_constant,
            'obst_point': vfl.obstacle_point,
            'obst_point2': vfl.obstacle_point_type2}
    # corr = {'patrol_circle': cn.patrouille_circulaire,
    #         'ligne': cn.ligne,
    #         'limite': cn.limite,
    #         'waypoint': cn.waypoint,
    #         'nul': cn.champ_nul,
    #         'obst_point': cn.obstacle_point}

    def __init__(self, behavior_info=None):
        self.info = behavior_info
        if self.info is not None:
            self.generate_behavior_from_info(self.info)
            self.is_simple = True
        else:
            self.generate_behavior_from_scratch()
            self.is_simple = False

    def generate_behavior_from_info(self, info):
        self.f_type = info.f_type
        self.behavior_id = info.behavior_id
        self.R = info.R
        self.K = info.K
        self.xa = info.xa
        self.ya = info.ya
        self.xb = info.xb
        self.yb = info.yb
        self.R = info.R
        self.K = info.K
        self.slowing_R = info.slowing_R
        self.slowing_K = info.slowing_K
        self.security = info.security
        self.effect_range = info.effect_range
        self.param_dict = {
            'patrol_circle': (self.xa, self.ya, self.K, self.R, self.slowing_R),
            'ligne': (self.xa, self.ya, self.xb, self.yb,
                      self.K, self.R, self.effect_range),
            'limite': (self.xa, self.ya, self.xb, self.yb, self.K, self.R,
                       self.security, self.slowing_R, self.slowing_K),
            'waypoint': (self.xa, self.ya, self.K),
            'constant': (self.xa, self.ya),
            'obst_point': (self.xa, self.ya, self.K, self.R,
                           self.security, self.slowing_R, self.slowing_K),
            'obst_point2': (self.xa, self.ya, self.K, self.R,
                            self.security, self.slowing_R, self.slowing_K)
        }
        self.params = self.param_dict[self.f_type]

        # self.behavior_function = lambda x: x
        self.behavior_function = type(self).corr[self.f_type]

    def generate_behavior_from_scratch(self):
        self.f_type = 'C#'
        self.behavior_id = 'C'
        self.R = 0
        self.K = 0
        self.xa = 0
        self.ya = 0
        self.xb = 0
        self.yb = 0
        self.R = 0
        self.K = 0
        self.slowing_R = 0
        self.slowing_K = 0
        self.security = 0
        self.effect_range = 0
        self.params = (self.xa, self.ya)
        self.behavior_function = type(self).corr['constant']

    def cmd_point(self, x, y):
        return self.behavior_function(x, y, *self.params)

    def __add__(self, other):
        summed_behavior = Behavior()
        summed_behavior.f_type = ','.join([self.f_type, other.f_type])
        summed_behavior.behavior_id = ','.join(
            [self.behavior_id, other.behavior_id])
        summed_behavior.cmd_point = lambda x, y: self.cmd_point(
            x, y) + other.cmd_point(x, y)
        return summed_behavior

    def __eq__(self, other):
        return self.behavior_id == other.behavior_id

##########################################################################
# Behavior INFO
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
# Behavior MANAGER
##########################################################################


class BehaviorManager():
    """Cette classe contient les outils pour creer,
     mettre a jour et publier les listes d'obstacles
     et d'objectifs, ainsi que les champs de vecteurs
     associes """

    def __init__(self):
        self.behavior_list = []
        self.champ_total = Behavior()

    # def add_behavior(self, new_behavior):
    #     # if not self.behavior_in_list(new_behavior):
    #     if new_behavior not in self.behavior_list:
    #         self.behavior_list.append(new_behavior)
    #         self.champ_total += new_behavior
    #     else:
    #         self.behavior_list.remove(new_behavior)
    #         self.recalc_champ_total()

    def handle_behavior(self, new_behavior, mode):
        """
        Handles a new behavior received according to the mode:
        :mode == add
            add the behavior only if NOT on the list. else do nothing
        :mode == update
            add the behavior if not on the list
            udpate the behavior if on the list
        :mode == remove
            remove the behavior from the list if in
        """
        if mode == 'add' and new_behavior not in self.behavior_list:
            self.behavior_list.append(new_behavior)
            self.champ_total += new_behavior
        elif mode == 'update':
            if new_behavior not in self.behavior_list:
                self.behavior_list.append(new_behavior)
                self.champ_total += new_behavior
            else:
                i = self.behavior_list.index(new_behavior)
                self.behavior_list[i] = new_behavior
        elif mode == 'remove' and new_behavior in self.behavior_list:
            self.behavior_list.remove(new_behavior)

        self.recalc_champ_total()

    def recalc_champ_total(self):
        self.champ_total = Behavior()
        for c in self.behavior_list:
            self.champ_total += c

    # def remove_behavior(self, new_behavior):
    #     for b in self.behavior_list:
    #         if new_behavior.behavior_id == b.behavior_id:
    #             self.behavior_list.remove(b)


##########################################################################
# TESTs
##########################################################################
def main_behavior():
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]

    # Behavior 0
    info0 = Behavior_info(f_type='constant', behavior_id='000', xa=0, ya=1)
    # info0 = Behavior_info('constant', '000', 0, 0, 0, 0, 0, -1)
    b0 = Behavior(info0)
    U0, V0 = b0.cmd_point(X, Y)
    plt.figure(0)
    plt.quiver(X, Y, U0, V0)

    # Behavior 1 - waypoint
    info1 = Behavior_info(f_type='waypoint', behavior_id='001', xa=5, ya=10,
                          xb=0, yb=0, K=1, R=0, slowing_R=0, slowing_K=0,
                          security='HIGH', effect_range=10)
    b1 = Behavior(info1)
    U1, V1 = b1.cmd_point(X, Y)
    plt.figure(1)
    plt.quiver(X, Y, U1, V1)

    # Behavior 2 - limite
    info2 = Behavior_info(f_type='limite', behavior_id='002', xa=-5, ya=0,
                          xb=5, yb=0, K=3, R=5, slowing_R=1, slowing_K=5,
                          security='MEDIUM', effect_range=10)
    b2 = Behavior(info2)
    U2, V2 = b2.cmd_point(X, Y)
    plt.figure(2)
    plt.quiver(X, Y, U2, V2)

    # Behavior 3 - patrol_circle
    info3 = Behavior_info(f_type='patrol_circle', behavior_id='003',
                          xa=0, ya=0, K=3, R=5)
    b3 = Behavior(info3)
    U3, V3 = b3.cmd_point(X, Y)
    plt.figure(3)
    plt.quiver(X, Y, U3, V3)

    # Behavior 4 - ligne
    info4 = Behavior_info(f_type='ligne', behavior_id='004',
                          xa=0, ya=0, xb=10, yb=0, K=4, R=5, effect_range=50)
    b4 = Behavior(info4)
    U4, V4 = b4.cmd_point(X, Y)
    plt.figure(4)
    plt.quiver(X, Y, U4, V4)

    # Behavior Total
    b = b1 + b2
    U, V = b.cmd_point(X, Y)
    plt.figure('Total')
    plt.quiver(X, Y, U, V)

    plt.show()


def main_manager():
    X, Y = np.mgrid[-100:100:40j, -100:100:40j]
    manager = BehaviorManager()

    info1 = Behavior_info(f_type='ligne', behavior_id='001', xa=0, ya=-100,
                          xb=100, yb=0,
                          K=1, R=20, slowing_R=1, slowing_K=5,
                          security='LOW', effect_range=40)
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

    manager.handle_behavior(b1, 'update')
    # manager.add_behavior(b2)
    manager.handle_behavior(b2, 'update')
    # manager.handle_behavior(b3, 'update')

    res = manager.champ_total.cmd_point(X, Y)
    print manager.champ_total.cmd_point(0, 0)

    U, V = res[0], res[1]
    plt.quiver(X, Y, U, V)
    plt.show()

if __name__ == '__main__':
    # main_behavior()
    main_manager()
