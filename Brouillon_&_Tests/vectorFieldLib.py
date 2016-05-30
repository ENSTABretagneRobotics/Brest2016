import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import det
from numpy.linalg import norm
from time import sleep

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


#########################################################
# FONCTIONS DE PROFIL: BAS NIVEAU
#########################################################


def gaussienne(x, L, D):
    """ Fonction gaussienne centree en D et de largeur L
    """
    return np.exp(-((x - D) * np.sqrt(np.log(2)) / L)**2)


def dirac(x, L, D, K=1):
    """
    Fonction dirac centree en D et de rayon L
    """
    if abs(x - D) > L:
        return 0
    else:
        return K

#########################################################
# FONCTIONS DE BAS NIVEAU DIRECTIONNELS
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
    """ Permet de generer un champ tournant vers la gauche (s=1)
            et centre en (a,b)"""
    x, y = translate(x, y, a, b)
    # vect = np.array()
    vect = normalize([-y, x])
    return vect


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

#########################################################
# FONCTION PROFILS de HAUT NIVEAU
#########################################################


def profil_point_security(x, y, a, b, K=1, R=1,
                          security='HIGH', slowing_R=0.5, slowing_K=5):
    """
    Cree un profil d'evitement d'obstacle ponctuel avec 3 niveaux de securites
    + SECURITY == HIGH:
        - Definit un profil en dirac autour du point(a,b)
        # Rq: sur mais pas smooth

    + SECURITY == MEDIUM: plus de calculs
        - un 'mur' de securite en dirac (pour freiner brusquement)
        - une gaussienne a l'interieur (pour 'smoother' la trajectoire)
        # Rq: sur mais plus de calculs

    + SECURITY == LOW:
        - Definit une gaussienne autour du point(a,b)
        # Rq: moins sur
    """
    x, y = translate(x, y, a, b)
    slowing_f = 0
    if security is 'HIGH':
        f = dirac(dist_point(x, y), R, 0, K)
    elif security is 'MEDIUM':
        slowing_f = dirac(dist_point(x, y), slowing_R, R, slowing_K)
        f = gaussienne(dist_point(x, y), R, 0)
    elif security is 'LOW':
        f = gaussienne(dist_point(x, y), R, 0)
    else:
        print security
        print 'WARNING: NO SECURITY HAS BEEN SET FOR THIS PROFILE'
        return
    return K * f + slowing_f


profil_point_security_M = np.vectorize(profil_point_security)
#########################################################
# FONCTION D'OBJECTIFS
#########################################################


def champ_constant(x, y, a, b):
    """
    Retourne un champ nul
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
    Defini un champ repulsif autour d'une zone en a, b
    """
    if security is 'HIGH':
        profil = profil_point_security_M(x, y, a, b, K, R, security=security)
    elif security is 'MEDIUM':
        profil = profil_point_security_M(
            x, y, a, b, K, R, slowing_R=slowing_R, slowing_K=slowing_K)
    direction = -dir_point(x, y, a, b)
    return profil * direction
