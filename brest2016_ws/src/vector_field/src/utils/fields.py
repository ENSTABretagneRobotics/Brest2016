# librairie des differents type de champs de vecteur

import numpy as np
from field_utils import *
from field_dist import *
from field_profil import *
from field_direction import *

#########################################################
# FONCTION D'OBJECTIFS
#########################################################


def champ_constant(x, y, a, b):
    """
    Retourne un champ constant
    """
    if type(x) in [float, int]:
        return [0, 0]
    elif type(x) is np.ndarray:
        return [a + np.zeros(x.shape), b + np.zeros(y.shape)]


def waypoint(x, y, a, b, K=1):
    """
    Defini un champ attire par un waypoint en a, b
    """
    return K * dir_point(x, y, a, b)


def obstacle_point(x, y, a, b, K=1, R=1,
                   security='HIGH', slowing_R=0.5, slowing_K=5):
    """
    Defini le champ repulsif autour d'un point en a, b
    Utilise le @profil_security
    """
    profil = profil_security_M(x, y, a, b, K=K, R=R, security=security,
                               slowing_R=slowing_R, slowing_K=slowing_K)
    direction = -dir_point(x, y, a, b)
    return profil * direction


def obstacle_point_type2(x, y, a, b, K=1, R=1,
                         security='HIGH', slowing_R=0.5, slowing_K=5):
    """
    Defini le champ repulsif autour d'un point en a, b
    Utilise le @profil_security
    """
    d = np.vectorize(dist_point)(x, y, a, b)
    f = gaussienne(d, R, 0)
    # profil 3
    bosse = gaussienne(d, slowing_R, R)

    Ca = -dir_point(x, y, a, b) * K * f
    Ct = dir_tournant(x, y, a, b) * K * bosse
    return Ca + Ct
    # profil = profil_security_M(x, y, a, b, K=K, R=R, security=security,
    #                            slowing_R=slowing_R, slowing_K=slowing_K)
    # direction = -dir_point(x, y, a, b)
    # return profil * direction


def limite(x, y, xa, ya, xb, yb, K=1, R=1,
           security='HIGH', slowing_R=0.5, slowing_K=5):
    """
    Defini le champ repulsif autour d'un segment [A, B].
    Utilise le @profil_security
    """
    profil_seg = profil_security_M(x, y, xa, ya, xb=xb, yb=yb, K=K, R=R,
                                   security=security,
                                   slowing_R=slowing_R, slowing_K=slowing_K,
                                   p_type='segment')
    dir_seg = - dir_segment(x, y, xa, ya, xb, yb)
    dir_ext = - dir_segment_extremity(x, y, xa, ya, xb, yb)
    return profil_seg * (dir_seg + dir_ext)


def ligne(x, y, xa, ya, xb, yb, K=1, R=1, effect_range=20):
    """
    Defini le champ d'une ligne attractive
    """
    d = np.vectorize(dist_droite)(x, y, xa, ya, xb, yb)
    f1 = gaussienne(d, 4 * R, 0)
    f2 = 1 - gaussienne(d, R, 0)
    # f2 = 1 - f1
    if type(x) in [int, np.float64, float, np.int64]:
        if abs(d) > effect_range:
            f1, f2 = 0, 0
    elif type(x) is np.ndarray:
        f1[abs(d) > effect_range] = 0
        f2[abs(d) > effect_range] = 0
    # f = 0 * f
    profil_tang = K * f1
    profil_norm = K * f2
    # Directions
    dir_tang = dir_segment(x, y, xa, ya, xb, yb, seg_type='tangent')
    dir_norm = dir_segment(x, y, xa, ya, xb, yb, seg_type='normal')
    return profil_tang * dir_tang + profil_norm * dir_norm


def patrouille_circulaire(x, y, a, b, K, R, turning_R=5):
    """
    Genere un cercle attractif tournant (a gauche)
    autour du point(a,b) et de rayon R
    """
    # profil 2
    # x, y = translate(x, y, a, b)
    d = np.vectorize(dist_point)(x, y, a, b)
    f = gaussienne(d, R, 0) - 0.5
    # profil 3
    bosse = gaussienne(d, turning_R, R)

    Ca = -dir_point(x, y, a, b) * K * f
    Ct = dir_tournant(x, y, a, b) * K * bosse
    return Ca + Ct


def projection(x, y, wind=90, theta=100):
    """"
    Projete le champ de vecteur sur des directions possibles
    pour un voilier, et un vent d'angle theta
    wind et theta sont en degres
    """
    U, V = x, y
    wind = angpipi(wind + 180)  # parce que le vent bloque l'oppose
    T = np.rad2deg(np.arctan2(V, U))
    # --------------------------------------------------------------------------
    if type(x) in [int, np.float64, float, np.int64]:
        U = np.ones((3, 3)) * x
        V = np.ones((3, 3)) * y
        Ucop = U.copy()
        Vcop = V.copy()
        T = np.ones((3, 3)) * T
    else:
        Ucop = U.copy()
        Vcop = V.copy()
    # --------------------------------------------------------------------------
    # borne inf et sup
    low = wind - theta / 2
    if low < -180:
        low += 360
        left = np.logical_and(-180 <= T, T < wind)
        left = np.logical_or(left, low < T)
    else:
        left = np.logical_and(low < T, T <= wind)
    upp = wind + theta / 2
    if upp > 180:
        upp -= 360
        right = np.logical_and(wind <= T, T <= 180)
        right = np.logical_or(right, T < upp)
    else:
        right = np.logical_and(wind <= T, T < upp)

    # --------------------------------------------------------------------------
    Ucop[left] = np.cos(np.deg2rad(wind - theta / 2))
    Vcop[left] = np.sin(np.deg2rad(wind - theta / 2))
    Ucop[right] = np.cos(np.deg2rad(wind + theta / 2))
    Vcop[right] = np.sin(np.deg2rad(wind + theta / 2))
    # --------------------------------------------------------------------------
    Ucop[U == 0] = 0
    Vcop[V == 0] = 0

    if type(x) in [int, np.float64, float, np.int64]:
        return Ucop[0, 0], Vcop[0, 0]
    else:
        return Ucop, Vcop
