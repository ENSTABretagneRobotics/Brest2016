import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import det
# from numpy.linalg import norm as np_norm
from time import sleep


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
    return vect / norm(vect)


def norm(vect):
    return np.sqrt(vect[0]**2 + vect[1]**2)


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
# FONCTIONS DE BAS NIVEAU COURBE
#########################################################


def gaussienne(x, L, D):
    """ Fonction gaussienne centree en D et de largeur L
    """
    return np.exp(-((x - D) * np.sqrt(np.log(2)) / L)**2)


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
    return vect_normed


def dir_tournant(x, y, a=0, b=0):
    """ Permet de generer un champ tournant vers la gauche (s=1)
            et centre en (a,b)"""
    x, y = translate(x, y, a, b)
    # vect = np.array()
    vect = normalize([-y, x])
    return vect


def dir_segment_perpendiculaire(x, y, xa, ya, xb, yb):
    """ Retourne le vecteur situe en (x,y) dirige perpendiculairement
    a un segment d'extremites A(xa, ya), B(xb, yb)
    """
    N = normalize([yb - ya, xa - xb])
    BM = np.array([y - yb, x - xb])
    BA = np.array([ya - yb, xa - xb])
    AB = -BA
    AM = np.array([y - ya, x - xa])
    cosa = np.dot(BM, BA)   # cos(alpha)
    cosb = np.dot(AB, AM)
    sa = np.cross(BM, BA)
    # sb = np.cross(AM, AB)
    if cosa > 0 and cosb > 0:   # interieur
        return -N if sa > 0 else N
    else:
        return np.array([0, 0])


def dir_segment_perpendiculaire_m(x, y, xa, ya, xb, yb):
    """
    Fonction similaire qui s'applique a un meshgrid
    """
    U = x.copy()
    V = y.copy()
    for row in range(len(x)):
        for col in range(len(x[0])):
            res = dir_segment_perpendiculaire(
                x[row][col], y[row][col], xa, ya, xb, yb)
            U[row][col] = res[0]
            V[row][col] = res[1]
    return U, V


def dir_segment_parallele(x, y, xa, ya, xb, yb):
    """ Genere des vecteurs paralleles segment d'extremites
    [(xa, ya), (xb, yb)]
    """
    N = normalize([xb - xa, yb - ya])
    BM = np.array([y - yb, x - xb])
    BA = np.array([ya - yb, xa - xb])
    AB = -BA
    AM = np.array([y - ya, x - xa])
    cosa = np.dot(BM, BA)   # cos(alpha)
    cosb = np.dot(AB, AM)
    if cosa > 0 and cosb > 0:
        return N
    else:
        return np.array([0, 0])


def dir_segment_parallele_m(x, y, xa, ya, xb, yb):
    U = x.copy()
    V = y.copy()
    for row in range(len(x)):
        for col in range(len(x[0])):
            res = dir_segment_parallele(
                x[row][col], y[row][col], xa, ya, xb, yb)
            U[row][col] = res[0]
            V[row][col] = res[1]
    return U, V


#########################################################
# FONCTIONS DE BAS NIVEAU - PROFILS D'INTENSITE
#########################################################


def profil0(s=1):
    """
    Renvoie un scalaire constant
    """
    return s

profil0_m = np.vectorize(profil0)


def profil1(x, y, a, b, s=1):
    """ Renvoit un scalaire inversement proportionnel a la dist au point (x,y)
        :s definit la repulsion/attraction
        (A appliquer a la fonction @point)
    """
    x, y = translate(x, y, a, b)
    if x == 0 and y == 0:
        return s
    else:
        return s / dist_point(x, y)

profil1_m = np.vectorize(profil1)


def profil2(x, y, a, b, r, s):
    """ Definit le potentiel d'un cercle centre en a,b
        (A appliquer a la fonction @point)
        :r definit le rayon du cercle
        :s definit la repulsion/attraction
    """
    x, y = translate(x, y, a, b)
    f = gaussienne(dist_point(x, y), r, 0) - 0.5
    # f = np.exp(-(dist_point(x, y)**2 * np.log(2)) / r**2) - 0.5
    return s * f

profil2_m = np.vectorize(profil2)


def profil3(x, y, a, b, r, s):
    """ Definit une gaussienne centree en r autour du point a,b et de largeur 1
    """
    x, y = translate(x, y, a, b)
    bosse = gaussienne(dist_point(x, y), 1, r)
    return s * bosse

profil3_m = np.vectorize(profil3)


def profil4(x, y, a, b, r, s):
    """ Definit une gaussienne centree au point a,b de largeur r
    """
    x, y = translate(x, y, a, b)
    f = gaussienne(dist_point(x, y), r, 0)
    return s * f

profil4_m = np.vectorize(profil4)


def profil5(x, y, xa, ya, xb, yb, r, s):
    d = dist_droite(x, y, xa, ya, xb, yb)
    f = gaussienne(d, r, 0)
    return s * f

profil5_m = np.vectorize(profil5)


def profil6(x, y, xa, ya, xb, yb, r, s, l=10):
    d = dist_droite(x, y, xa, ya, xb, yb)
    f = 1 - gaussienne(d, r, 0)
    if abs(d) > l:
        f = 0 * f
    return s * f

profil6_m = np.vectorize(profil6)

#########################################################
# FONCTIONS DE HAUT NIVEAU - GENERATION DE CHAMP
#########################################################


def champ_constant(x, y, a, b, s):
    u = 0 * x + a
    v = 0 * y + b
    return s * normalize([u, v])


def point_courte_portee(x, y, a, b, s=1, r=2):
    """
    Genere un champ autour d'un point(a,b) de rayon r
    : attractif __ s > 0
    : repulstif __ s < 0

    """
    return dir_point(x, y, a, b) * profil4_m(x, y, a, b, r=r, s=s)


def cercle_attractif(x, y, a, b, r, s=1):
    return dir_point(x, y, a, b) * profil2_m(x, y, a, b, r, s)


def champ_tournant1(x, y, a, b, r, s=1):
    """
    Genere un champ tournant partout et nul sur un cercle centre
    en a,b et de rayon r
    """
    return dir_tournant(x, y, a, b) * profil2_m(x, y, a, b, r, s)


def champ_tournant2(x, y, a, b, r, s=1):
    """
    Genere un champ tournant uniquement sur
    un cercle centre en a,b et de rayon r (nul ailleurs)
    """
    return dir_tournant(x, y, a, b) * profil3_m(x, y, a, b, r, s)


def champ_segment_courte_portee_perpendiculaire(x, y, xa, ya, xb, yb, s=1, r=1):
    """
    Genere un champ perpendiculaire a un segment [A,B]
    Les normes suivent une gaussienne centree sur ce segment et de largeur r
    """
    dirc = dir_segment_perpendiculaire_m(x, y, xa, ya, xb, yb)
    f = profil5_m(x, y, xa, ya, xb, yb, r, s)
    return dirc * f


def champ_segment_longue_portee_perpendiculaire(x, y, xa, ya, xb, yb, s=1, l=10):
    """
    Genere un champ perpendiculaire a un segment [A,B]
    Les normes suivent une (1-gaussienne) centree sur ce segment
    """
    dirc = dir_segment_perpendiculaire_m(x, y, xa, ya, xb, yb)
    f = profil6_m(x, y, xa, ya, xb, yb, 1, s, l)
    return dirc * f


def champ_segment_courte_portee_parallele(x, y, xa, ya, xb, yb, s=1):
    """
    Genere un champ perpendiculaire a un segment [A,B]
    Les normes suivent une gaussienne centree sur ce segment
    """
    dirc = dir_segment_parallele_m(x, y, xa, ya, xb, yb)
    f = profil5_m(x, y, xa, ya, xb, yb, 5, s)
    return dirc * f


#########################################################
# FONCTION D'OBJECTIFS                                  #
#########################################################


def waypoint(x, y, a, b, s=-1):
    """
    Defini un champ attire par un waypoint en a,b
    """
    return s * dir_point(x, y, a, b)


def limite(x, y, xa, ya, xb, yb, s=1, r=1):
    """
    Definit une limite repulsive
    """
    seg = champ_segment_courte_portee_perpendiculaire(
        x, y, xa, ya, xb, yb, s, r=r)
    Pa = point_courte_portee(x, y, xa, ya, -s / 1., r=r)
    Pb = point_courte_portee(x, y, xb, yb, -s / 1., r=r)
    return Pa + seg + Pb


def ligne(x, y, xa, ya, xb, yb, s=1, l=10):
    """
    Definit une ligne attractive et orientee de A vers B
    """
    seg_attr = champ_segment_longue_portee_perpendiculaire(
        x, y, xa, ya, xb, yb, s=1, l=l)
    seg_cour = champ_segment_courte_portee_parallele(x, y, xa, ya, xb, yb, s=1)
    return seg_cour + seg_attr


def patrouille_circulaire(x, y, a, b, r, s=1):
    """
    Genere un cercle attractif tournant autour de a,b et de rayon r
    """
    Uc, Vc = cercle_attractif(x, y, a, b, r=r, s=s)
    Ut, Vt = champ_tournant2(x, y, a, b, r=r, s=-s)
    return [Uc + Ut, Vc + Vt]
#########################################################
# MAIN                                                   *****************
#########################################################


def champ_carre(X, Y, L):
    Us1, Vs1 = champ_segment_courte_portee_perpendiculaire(
        X, Y, -L, -L, -L, L, -1)
    Us2, Vs2 = champ_segment_courte_portee_perpendiculaire(
        X, Y, -L, L, L, L, -1)
    Us3, Vs3 = champ_segment_courte_portee_perpendiculaire(
        X, Y, L, L, L, -L, -1)
    Us4, Vs4 = champ_segment_courte_portee_perpendiculaire(
        X, Y, L, -L, -L, -L, -1)
    Utot = Us1 + Us2 + Us3 + Us4
    Vtot = Vs1 + Vs2 + Vs3 + Vs4
    return Utot, Vtot
    # return Us2, Vs2


def champ_carre_2P(X, Y, L, s=-1):
    Us1, Vs1 = limite(X, Y, -L, -L, -L, L, s)
    Us2, Vs2 = limite(X, Y, -L, L, L, L, s)
    Us3, Vs3 = limite(X, Y, L, L, L, -L, s)
    Us4, Vs4 = limite(X, Y, L, -L, -L, -L, s)
    Utot = Us1 + Us2 + Us3 + Us4
    Vtot = Vs1 + Vs2 + Vs3 + Vs4
    return Utot, Vtot
    # return Us2, Vs2


def points(X, Y, s=1):
    Up1, Vp1 = point_courte_portee(X, Y, 10, 10, s)
    Up2, Vp2 = point_courte_portee(X, Y, -10, -10, s)
    Up3, Vp3 = point_courte_portee(X, Y, 10, -10, s)
    Up4, Vp4 = point_courte_portee(X, Y, -10, 10, s)
    Utot = Up1 + Up2 + Up3 + Up4
    Vtot = Vp1 + Vp2 + Vp3 + Vp4
    return Utot, Vtot


def main():
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]
    Up2, Vp2 = points(X, Y, 1)
    Ucar, Vcar = champ_carre(X, Y, 20)
    plt.figure('total')
    plt.ion()
    for a in range(-10, 15):
        plt.cla()
        # Uc, Vc = cercle_attractif(X, Y, a, a, r=5, s=1)
        # Ut, Vt = champ_tournant2(X, Y, a, a, r=5, s=-1)
        Ut, Vt = waypoint(X, Y, a, a, s=-1)
        Utot, Vtot = Ucar + Ut + Up2, Vcar + Vt + Vp2

        plt.hold(True)
        plt.quiver(X, Y, Utot, Vtot)
        # plt.colorbar()
        plt.plot([10, 10, -10, -10, a], [10, -10, 10, -10, a], 'ro')
        plt.axis('equal')
        plt.draw()
        sleep(0.1)


def main2():
    # Meshgrid
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]
    # x, y = 0, 2

    # Test de calculs de champs
    a, b = 0, 0
    # a1, b1 = 10, 10
    # xa, ya, xb, yb = -5, -6, -5, -2  # ligne verticale
    # xa, ya, xb, yb = 1, 1, 4, 4   # ligne oblique

    # xa, ya, xb, yb = 0, 0, 10, 0   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, 10, 10   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, 0, 10   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, -10, 10   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, -10, 0   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, -10, -10   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, 0, -10   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, 10, -10   # ligne horizontale

    U, V = champ_constant(X, Y, 7, 2, 0.5)
    # print champ_constant(x, y, 7, 2, 0.5)
    Up, Vp = point_courte_portee(X, Y, a, b, s=1)
    Up2, Vp2 = points(X, Y, 2)
    # print point_courte_portee(x, y, a, b, s=1)
    Uc, Vc = cercle_attractif(X, Y, a, b, r=5, s=1)
    # print cercle_attractif(x, y, a, b, r=5, s=1)
    Ut, Vt = champ_tournant2(X, Y, a, b, r=5, s=-1)
    # print champ_tournant2(x, y, a, b, r=5, s=-1)
    Uw, Vw = waypoint(X, Y, a, b, s=-1)

    # print "dir segment"
    # Us, Vs = champ_segment_courte_portee_perpendiculaire(X, Y, xa, ya, xb, yb, 1)
    Ucar, Vcar = champ_carre(X, Y, 20)
    # for x in xrange(-5, 5):
    #     for y in xrange(-5, 5):
    #         print x, y, dir_segment_perpendiculaire(x, y, 1, 1, 4, 4)
    # print dir_segment_perpendiculaire(0, 0, 1, 1, 4, 4)
    # print dir_segment_perpendiculaire(-1, 4, 1, 1, 4, 4)
    # print dir_segment_perpendiculaire(-1, 1, 1, 1, 4, 4)
    # print dir_segment_perpendiculaire(1, 2, 1, 1, 4, 4)
    # print dir_segment_perpendiculaire(3, 1, 1, 1, 4, 4)
    # print dir_segment_perpendiculaire(2, -2, 1, 1, 4, 4)
    # print Us, Vs

    # Somme des champs
    # Utot, Vtot = 0 * U + Up + Uc + Ut, 0 * V + Vp + Vc + Vt
    # Utot, Vtot = Up + Us + Uc, Vp + Vs + Vc
    # print Utot[0][0], Vtot[0][0]
    Utot, Vtot = Uc + Ucar + Ut + Up2, Vc + Vcar + Vt + Vp2

    #############################
    # Affichage

    # plt.figure('segment')
    # plt.hold(True)
    # plt.quiver(X, Y, Us, Vs)
    # plt.plot([xa, xb], [ya, yb], '-ro')
    # plt.axis('equal')

    plt.figure('carre')
    plt.hold(True)
    plt.quiver(X, Y, Ucar, Vcar)
    plt.axis('equal')

    plt.figure('champ_constant')
    plt.quiver(X, Y, U, V)

    plt.figure('point')
    plt.quiver(X, Y, Up, Vp)

    plt.figure('cercle')
    plt.quiver(X, Y, Uc, Vc)
    plt.axis('equal')

    plt.figure('tournant')
    plt.quiver(X, Y, Ut, Vt)
    plt.axis('equal')

    plt.figure('waypoint')
    plt.quiver(X, Y, Uw, Vw)
    plt.axis('equal')

    plt.figure('total')
    plt.quiver(X, Y, Utot, Vtot)
    plt.axis('equal')

    plt.hold(False)
    plt.show()


def main3():
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]
    xa, ya, xb, yb = 0, 0, 10, 0   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, 10, 10   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, 0, 10   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, -10, 10   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, -10, 0   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, -10, -10   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, 0, -10   # ligne horizontale
    # xa, ya, xb, yb = 0, 0, 10, -10   # ligne horizontale
    U, V = limite(X, Y, xa, ya, xb, yb, s=1)
    Ucar, Vcar = champ_carre_2P(X, Y, 10, s=-4)
    Uc, Vc = cercle_attractif(X, Y, xa, ya, r=5, s=1)
    Utot, Vtot = Ucar + Uc, Vcar + Vc
    plt.quiver(X, Y, Utot, Vtot)
    plt.axis('equal')
    plt.show()


def champ_carre_tournant(X, Y):
    xa, ya, xb, yb = 0, 0, 10, 0
    U1, V1 = ligne(X, Y, xa, ya, xb, yb, s=5)
    Uc1, Vc1 = waypoint(X, Y, xb, yb, s=-1)
    xa, ya, xb, yb = 10, 0, 10, 10
    U2, V2 = ligne(X, Y, xa, ya, xb, yb, s=5)
    Uc2, Vc2 = waypoint(X, Y, xb, yb, s=-1)
    xa, ya, xb, yb = 10, 10, 0, 10
    U3, V3 = ligne(X, Y, xa, ya, xb, yb, s=5)
    Uc3, Vc3 = waypoint(X, Y, xb, yb, s=-1)
    xa, ya, xb, yb = 0, 10, 0, 0
    U4, V4 = ligne(X, Y, xa, ya, xb, yb, s=5)
    Uc4, Vc4 = waypoint(X, Y, xb, yb, s=-1)
    Utot = U1 + U2 + U3 + U4
    Vtot = V1 + V2 + V3 + V4
    return Utot, Vtot


def main4():
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]
    U, V = champ_carre_tournant(X, Y)
    plt.quiver(X, Y, U, V)
    plt.axis('equal')
    plt.show()


def main5():
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]
    # xa, ya, xb, yb = 0, 0, 5, 5
    # U0, V0 = champ_constant(X, Y, 0, 1, s=1)
    a, b = 3, 5
    Uc, Vc = cercle_attractif(X, Y, a, b, r=5, s=1)
    Ut, Vt = champ_tournant2(X, Y, a, b, r=5, s=-1)

    plt.figure(1)
    plt.quiver(X, Y, Uc, Vc)
    plt.axis('equal')

    plt.figure(2)
    plt.quiver(X, Y, Ut, Vt)
    plt.axis('equal')

    plt.figure(3)
    plt.quiver(X, Y, Ut + Uc, Vt + Vc)
    plt.axis('equal')
    plt.show()


def test_objectifs():
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]
    U0, V0 = patrouille_circulaire(X, Y, -10, -10, 5)
    U1, V1 = waypoint(X, Y, -10, 10)
    U2, V2 = limite(X, Y, -7, 0, 6, 4, s=-4, r=3)
    U3, V3 = ligne(X, Y, -10, 10, -10, -10, l=30)

    for i, k in enumerate([(U0, V0), (U1, V1), (U2, V2), (U3, V3)]):
        plt.figure(i)
        plt.quiver(X, Y, k[0], k[1])
        plt.axis('equal')

    U = 1 * U0 + 1 * U1 + 0 * U2 + 1 * U3
    V = 1 * V0 + 1 * V1 + 0 * V2 + 1 * V3
    plt.figure('total')
    plt.quiver(X, Y, U, V)
    plt.axis('equal')
    plt.show()


if __name__ == '__main__':
    # main()
    # main2()
    # main3()
    # main4()
    # main5()
    test_objectifs()
