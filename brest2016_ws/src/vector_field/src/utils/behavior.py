# defini les differents types de behavior
# et les informations qu il doivent contenir

# -*- coding: utf-8 -*-
# import champs_numpy as cn
import fields as vfl
import numpy as np


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
        self.projection = False
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

    def get_field(self, x, y, wind, theta, normalize=False):
        U, V = self.cmd_point(x, y)
        if normalize:
            N = (U**2 + V**2)**0.5
            U, V = U / N, V / N
        if self.projection:
            return vfl.projection(U, V, wind, theta)
        else:
            return np.array([U, V])

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
