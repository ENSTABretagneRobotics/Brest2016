# librairie des differents type de champs de vecteur

import numpy as np
from numpy.linalg import det
from numpy.linalg import norm
from field_utils import zone_segment


def dist_point(x, y, a=0, b=0):
    """
    Retourne la distance par rapport au point (a,b)
    """
    return np.sqrt((x - a)**2 + (y - b)**2)


def dist_droite(x, y, xa, ya, xb, yb):
    """
    Renvoie la distance d'un point (x,y) par rapport a la droite definie
     par A(xa,ya) et B(xb,yb)
    """
    if xa == xb and ya == yb:
        return 0
    ab = np.array([xb - xa, yb - ya])
    am = np.array([x - xa, y - ya])
    e = det([ab / norm(ab), am])
    return e


def dist_segment(x, y, xa, ya, xb, yb):
    """
    Renvoie la distance d'un point (x,y) par rapport au segment definie
    par A(xa,ya) et B(xb,yb).
    Renvoie la distance par rapport aux extremites en dehors du segment.
    """
    # Zones
    zone = zone_segment(x, y, xa, ya, xb, yb)
    if zone.startswith('IN'):
        return dist_droite(x, y, xa, ya, xb, yb)
    elif zone == 'EX_L':
        return dist_point(x, y, xa, ya)
    elif zone == 'EX_R':
        return dist_point(x, y, xb, yb)
    else:
        print 'WRONG ZONE'
