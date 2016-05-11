import champs_numpy as cn
import numpy as np
import matplotlib.pyplot as plt


class Behavior(object):
    """A behavior generates a vector field it can be added to others"""

    corr = {'patrol_circle': cn.patrouille_circulaire,
            'ligne': cn.ligne,
            'limite': cn.limite,
            'waypoint': cn.waypoint}

    def __init__(self, behavior_info=None):
        self.info = behavior_info
        if self.info is not None:
            self.generate_behavior_from_info(self.info)
            self.is_simple = True
        else:
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

    def behavior_point(self, x, y):
        return self.behavior_function(x, y, *self.params)

    def __add__(self, other):
        summed_behavior = Behavior()
        summed_behavior.type = ','.join([self.type, other.type])
        summed_behavior.behavior_id = ','.join([self.behavior_id, other.behavior_id])
        summed_behavior.behavior_point = lambda x, y: self.behavior_point(
            x, y) + other.behavior_point(x, y)
        return summed_behavior


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


if __name__ == '__main__':
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]

    # Behavior 1
    info1 = Behavior_info('waypoint', '001', -5, 5, 0, 0, 0, -1)
    b1 = Behavior(info1)
    U1, V1 = b1.behavior_point(X, Y)
    plt.figure(1)
    plt.quiver(X, Y, U1, V1)

    # Behavior 2
    info2 = Behavior_info('limite', '002', 0, 0, 3, 3, 1, -3)
    b2 = Behavior(info2)
    U2, V2 = b2.behavior_point(X, Y)
    plt.figure(2)
    plt.quiver(X, Y, U2, V2)

    # Behavior Total
    b = b1 + b2
    U, V = b.behavior_point(X, Y)
    plt.figure('Total')
    plt.quiver(X, Y, U, V)

    plt.show()
