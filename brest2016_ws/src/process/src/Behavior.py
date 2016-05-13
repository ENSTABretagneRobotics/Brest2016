# -*- coding: utf-8 -*-
import champs_numpy as cn
import numpy as np
import matplotlib.pyplot as plt

##########################################################################
# Behavior
##########################################################################


class Behavior(object):
    """A behavior generates a vector field it can be added to others"""

    corr = {'patrol_circle': cn.patrouille_circulaire,
            'ligne': cn.ligne,
            'limite': cn.limite,
            'waypoint': cn.waypoint,
            'nul': cn.champ_nul}

    def __init__(self, behavior_info=None):
        self.info = behavior_info
        if self.info is not None:
            self.generate_behavior_from_info(self.info)
            self.is_simple = True
        else:
            self.generate_behavior_from_scratch()
            self.is_simple = False

    def generate_behavior_from_info(self, info):
        self.type = info.type
        self.behavior_id = info.behavior_id
        self.r = info.r
        self.s = info.s
        if self.type in ['ligne', 'limite']:
            self.xa = info.xa
            self.ya = info.ya
            self.xb = info.xb
            self.yb = info.yb
            self.params = (self.xa, self.ya, self.xb, self.yb, self.s, self.r)
        elif self.type == 'waypoint':
            self.x = info.xa
            self.y = info.ya
            self.params = (self.x, self.y, self.s)
        elif self.type == 'patrol_circle':
            self.x = info.xa
            self.y = info.ya
            self.params = (self.x, self.y, self.r, self.s)

        # self.behavior_function = lambda x: x
        self.behavior_function = type(self).corr[self.type]

    def generate_behavior_from_scratch(self):
        self.type = 'C#'
        self.behavior_id = "C"
        self.r = 0
        self.s = 0
        self.xa, self.ya = 0, 0
        self.xb, self.yb = 0, 0
        self.x, self.y = 0, 0
        self.params = ()
        self.behavior_function = type(self).corr['nul']

    def cmd_point(self, x, y):
        return self.behavior_function(x, y, *self.params)

    def __add__(self, other):
        summed_behavior = Behavior()
        summed_behavior.type = ','.join([self.type, other.type])
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
    def __init__(self, type, behavior_id, xa, ya, xb, yb, r, s):
        self.type = type
        self.behavior_id = behavior_id
        self.xa = xa
        self.ya = ya
        self.xb = xb
        self.yb = yb
        self.r = r
        self.s = s

##########################################################################
# Behavior MANAGER
##########################################################################


class BehaviorManager():
    """Cette classe contient les outils pour créer,
     mettre à jour et publier les listes d'obstacles
     et d'objectifs, ainsi que les champs de vecteurs
     associés """

    def __init__(self):
        self.behavior_list = []
        self.champ_total = Behavior()

    def add_behavior(self, new_behavior):
        if not self.behavior_in_list(new_behavior):
            self.behavior_list.append(new_behavior)
            self.champ_total += new_behavior
        else:
            self.behavior_list.remove(new_behavior)
            self.recalc_champ_total()

    def behavior_in_list(self, behavior):
        for b in self.behavior_list:
            if behavior.behavior_id == b.behavior_id:
                return True
        return False

    def recalc_champ_total(self):
        self.champ_total = Behavior()
        for c in self.behavior_list:
            self.champ_total += c

    def remove_behavior(self, new_behavior):
        for b in self.behavior_list:
            if new_behavior.behavior_id == b.behavior_id:
                self.behavior_list.remove(b)


##########################################################################
# TESTs
##########################################################################
def main_behavior():
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]

    # Behavior 0
    info0 = Behavior_info('nu', '000', 0, 0, 0, 0, 0, -1)
    b0 = Behavior()
    U0, V0 = b0.cmd_point(X, Y)
    plt.figure(0)
    plt.quiver(X, Y, U0, V0)

    # Behavior 1 - waypoint
    info1 = Behavior_info('waypoint', '001', 0, 0, 0, 0, 0, -1)
    b1 = Behavior(info1)
    U1, V1 = b1.cmd_point(X, Y)
    plt.figure(1)
    plt.quiver(X, Y, U1, V1)

    # Behavior 2 - limite
    info2 = Behavior_info('limite', '002', 0, 0, 3, 0, 1, -3)
    b2 = Behavior(info2)
    U2, V2 = b2.cmd_point(X, Y)
    plt.figure(2)
    plt.quiver(X, Y, U2, V2)

    # Behavior 3 - patrol_circle
    info3 = Behavior_info('patrol_circle', '003', 0, 0, 3, 3, 1, 1)
    b3 = Behavior(info3)
    U3, V3 = b3.cmd_point(X, Y)
    plt.figure(3)
    plt.quiver(X, Y, U3, V3)

    # Behavior Total
    b = b1 + b2
    U, V = b.cmd_point(X, Y)
    plt.figure('Total')
    plt.quiver(X, Y, U, V)

    plt.show()


def main_manager():
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]
    manager = BehaviorManager()

    # Behavior 2 - limite
    info2 = Behavior_info('limite', '002', 0, 0, 3, 3, 1, -3)
    b2 = Behavior(info2)
    # Behavior 3 - patrol_circle
    info3 = Behavior_info('patrol_circle', '003', 0, 0, 3, 3, 1, 1)
    b3 = Behavior(info3)

    manager.add_behavior(b3)
    manager.add_behavior(b2)

    res = manager.champ_total.cmd_point(X, Y)
    manager.champ_total.cmd_point(0, 0)

    U, V = res[0], res[1]
    plt.quiver(X, Y, U, V)
    plt.show()

if __name__ == '__main__':
    main_behavior()
    # main_manager()
