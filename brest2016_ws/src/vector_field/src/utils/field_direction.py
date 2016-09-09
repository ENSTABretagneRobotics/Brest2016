#########################################################
# FONCTIONS DIRECTIONS
#########################################################
import numpy as np
from field_utils import *


def dir_point(x, y, a, b):
    """ Retourne un (champ de) vecteur pointant vers le point (a,b)
        Ce (champ de) vecteur est norme
        Vecteur en (x,y) dirige vers (a,b)
    """
    x, y = translate(x, y, a, b)
    norm = np.sqrt(x**2 + y**2)
    vect_normed = np.array([x, y]) / norm
    return -vect_normed


def dir_tournant(x, y, a=0, b=0):
    """ Permet de generer un champ tournant vers la gauche
            et centre en (a,b)"""
    x, y = translate(x, y, a, b)
    if type(x) in [int, float, np.float64]:
        return normalize([-y, x])
    # vect = np.array()
    z = np.dstack((-y, x))
    U, V = np.dsplit(np.apply_along_axis(normalize, axis=2, arr=z), 2)
    U = np.apply_along_axis(lambda x: x[0], axis=2, arr=U)
    V = np.apply_along_axis(lambda x: x[0], axis=2, arr=V)
    # print vect, '-' * 100
    return np.array([U, V])


def dir_segment(x, y, xa, ya, xb, yb, seg_type='normal'):
    """
    Retourne un champ dirige vers un segment:
    EX_R : [0, 0]
    EX_L : [0, 0]
    IN_R : -N ou T
    IN_L : N ou T
    """
    # Zones
    zoneX = zone_segment_M(x, y, xa, ya, xb, yb)
    zoneY = zoneX.copy()

    # Vecteurs de direction
    N = normalize([yb - ya, xa - xb])   # vecteur normal
    T = normalize([xb - xa, yb - ya])   # vecteur tangent
    seg_dict = {'normal': [-N, np.zeros(2), N, np.zeros(2)],
                'tangent': [T, np.zeros(2), T, np.zeros(2)]}
    zone_cor = {'IN_R': seg_dict[seg_type][0],
                'EX_R': seg_dict[seg_type][3],
                'IN_L': seg_dict[seg_type][2],
                'EX_L': seg_dict[seg_type][1]}
    for k, v in zone_cor.items():
        z = zoneX == k
        zoneX[z] = v[0]
        zoneY[z] = v[1]
    zoneX = zoneX.astype(float)
    zoneY = zoneY.astype(float)
    return np.array([zoneX, zoneY])


def dir_segment_extremity(x, y, xa, ya, xb, yb):
    """
    Retourne un champ dirige vers les extremites d'un segment:
    EX_R : dir_point xb, yb
    EX_L : dir_point xa, ya
    IN_R : [0, 0]
    IN_L : [0, 0]
    """
    # Zones
    zoneX = zone_segment_M(x, y, xa, ya, xb, yb)
    zoneY = zoneX.copy()

    # Vecteurs de direction
    dirA = dir_point(x, y, xa, ya)
    dirB = dir_point(x, y, xb, yb)
    zone_cor_in = {'IN_R': np.zeros(2),
                   'IN_L': np.zeros(2)}
    zone_cor_ex = {'EX_R': dirB,
                   'EX_L': dirA}
    for k, v in zone_cor_ex.items():
        z = zoneX == k
        zoneX[z] = v[0][z] if type(x) is np.ndarray else v[0]
        zoneY[z] = v[1][z] if type(x) is np.ndarray else v[1]
    for k, v in zone_cor_in.items():
        z = zoneX == k
        zoneX[z] = v[0]
        zoneY[z] = v[1]
    zoneX = zoneX.astype(float)
    zoneY = zoneY.astype(float)
    return np.array([zoneX, zoneY])
