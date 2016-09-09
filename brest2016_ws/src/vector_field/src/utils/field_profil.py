#########################################################
# FONCTIONS DE PROFIL: BAS NIVEAU
#########################################################
import numpy as np
from field_dist import *


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
