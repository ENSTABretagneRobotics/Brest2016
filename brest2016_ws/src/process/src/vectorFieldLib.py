# librairie des differents type de champs de vecteur

import numpy as np
from numpy.linalg import det
from numpy.linalg import norm

#########################################################
# FONCTIONS DE BASE
#########################################################


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

#########################################################
# FONCTIONS DE PROFIL: BAS NIVEAU
#########################################################


def gaussienne(x, L, D):
    """ Fonction gaussienne centree en D et de largeur L
    """
    return np.exp(-((x - D) * np.sqrt(np.log(2)) / L)**2)


def dirac(x, L, D, K=1):
    """
    Fonction dirac centree en D et de largeur L
    """
    if abs(x - D) > L:
        return 0
    else:
        return K

#########################################################
# FONCTIONS DIRECTIONS
#########################################################


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
#########################################################
# FONCTION PROFILS de HAUT NIVEAU
#########################################################


def profil_security(x, y, xa, ya, xb=0, yb=0, K=1, R=1,
                    security='HIGH', slowing_R=0.5, slowing_K=5,
                    p_type='point'):
    """
    Cree un profil d'evitement d'obstacle avec 3 niveaux de securites
    + SECURITY == HIGH:
        - Definit un profil en dirac autour du point/segment
        # Rq: sur mais pas smooth

    + SECURITY == MEDIUM: plus de calculs
        - un 'mur' de securite en dirac (pour freiner brusquement)
        - une gaussienne a l'interieur (pour 'smoother' la trajectoire)
        # Rq: sur mais plus de calculs

    + SECURITY == LOW:
        - Definit une gaussienne autour du point/segment
        # Rq: moins sur

    Parameters
    -----------
    security : string
        Permet de choisir le niveau de securite
    p_type : string
        Permet de choisir la forme sur laquelle le profil s'appliquera:
        - point
        - segment
        - ligne
    slowing_R : float
        Rayon du cercle exterieur de freinage
    slowing_K : float
        Force du cercle exterieur de freinage
    """
    # x, y = translate(x, y, xa, ya)
    # fonction a utiliser pour une droite ou un segment
    f_dict = {'point': [dist_point, (x, y, xa, ya)],
              'ligne': [dist_droite, (x, y, xa, ya, xb, yb)],
              'segment': [dist_segment, (x, y, xa, ya, xb, yb)]}
    dist_f = f_dict[p_type][0]
    dist_param = f_dict[p_type][1]
    slowing_f = 0
    if security == 'HIGH':
        # print 'high security has been chosen'
        f = dirac(abs(dist_f(*dist_param)), R, 0, K)
    elif security == 'MEDIUM':
        # print 'medium security has been chosen'
        slowing_f = dirac(abs(dist_f(*dist_param)), slowing_R, R, slowing_K)
        f = gaussienne(abs(dist_f(*dist_param)), R, 0)
    elif security == 'LOW':
        # print 'low security has been chosen'
        f = gaussienne(abs(dist_f(*dist_param)), R, 0)
    else:
        print security
        print 'WARNING: NO SECURITY HAS BEEN SET FOR THIS PROFILE'
        return
    return K * f + slowing_f


profil_security_M = np.vectorize(profil_security)


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
    f1 = gaussienne(d, R, 0)
    f2 = 1 - f1
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


def patrouille_circulaire(x, y, a, b, K, R, turning_R=2):
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
