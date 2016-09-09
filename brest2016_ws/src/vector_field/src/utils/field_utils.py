"""
Fonction utiles aux champs de vecteurs
"""

import numpy as np
from numpy.linalg import norm


def translate(x, y, a, b):
    """
    Deplace le point (x,y) dans le repere d'origine (a,b)
    """
    return [x - a, y - b]


def rotate(x, y, theta):
    """
    Fait pivoter le point (x,y) autour du point d'origine du repere
    """
    rot = np.array([[np.cos(theta), -np.sin(theta)],
                    [np.sin(theta), np.cos(theta)]])
    return np.dot(rot, [x, y])


def normalize(vect):
    """
    Retourne le vecteur normalise
    """
    # if norm(vect) == 0:
    #     return vect
    return vect / norm(vect)


def angpipi(ang):
    """
    Retourne un angle compris entre [-pi, pi]
    """
    while ang <= -180:
        ang += 360
    while ang > 180:
        ang -= 360
    return ang


def zone_segment(x, y, xa, ya, xb, yb):
    """
    Permet de situer un point dans une zone adjacente au segment
    Returns:
    EX_R : EXTERIEUR aux limites du segment, sur la DROITE
    EX_L : EXTERIEUR aux limites du segment, sur la GAUCHE
    IN_R : INTERIEUR aux limites du segment, sur la DROITE
    IN_L : INTERIEUR aux limites du segment, sur la GAUCHE

    """
    BM = np.array([y - yb, x - xb])
    BA = np.array([ya - yb, xa - xb])
    AB = -BA
    AM = np.array([y - ya, x - xa])
    cosa = np.dot(AB, AM)   # cos(alpha)
    cosb = np.dot(BA, BM)
    sa = np.cross(BM, BA)
    # sb = np.cross(AM, AB)
    if cosa >= 0 and cosb >= 0:   # interieur
        return 'IN_R' if sa > 0 else 'IN_L'
    else:
        if cosb < 0:
            return 'EX_R'
        elif cosa < 0:
            return 'EX_L'

zone_segment_M = np.vectorize(zone_segment)
