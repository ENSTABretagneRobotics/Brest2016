from vectorFieldLib import *
import matplotlib.pyplot as plt
import numpy as np


def rotation2(x, y, a, b, K=1, R=1):
    """
    Genere un champ rotatif (a gauche)
    autour du point(a,b) et de rayon R
    """
    d = dist_point(x, y, a, b)
    f = np.vectorize(dirac)(d, R, 0)
    rot = dir_tournant(x, y, a, b)
    pnt = dir_point(x, y, a, b)

    return f * (rot + pnt)


def projectiontest(wind):
    X, Y = np.mgrid[-100:100:40j, -100:100:40j]
    # U, V = patrouille_circulaire(X, Y, 0, 0, 1, 30, 20)
    # U, V = waypoint(X, Y, 0, 0)
    # U, V = limite(X, Y, -40, 0, 40, 0, K=1, R=3, security='LOW')

    # U1, V1 = limite(X, Y, -40, 0, 40, 0, K=5, R=10, security='LOW')
    # U2, V2 = waypoint(X, Y, 70, 70)
    # U, V = U1 + U2, V1 + V2
    U, V = ligne(X, Y, -60, -0, 60, 0, K=1, R=5, effect_range=100)
    plt.figure('before' + str(wind))
    plt.quiver(X, Y, U, V)
    plt.axis('equal')
    plt.axis([-120, 120, -120, 120])

    # wind = 90
    theta = 70
    T = np.rad2deg(np.arctan2(V, U))
    # T = np.arctan(V / U)
    # print T[15:25, 15:25]
    print T[0, 0], T[39, 39]

    Ucop = U.copy()
    Vcop = V.copy()
    Ucop[np.logical_and(wind - theta / 2 < T, T < wind)
         ] = np.cos(np.deg2rad(wind - theta / 2))
    Vcop[np.logical_and(wind - theta / 2 < T, T < wind)
         ] = np.sin(np.deg2rad(wind - theta / 2))

    Ucop[np.logical_and(wind <= T, T < wind + theta / 2)
         ] = np.cos(np.deg2rad(wind + theta / 2))
    Vcop[np.logical_and(wind <= T, T < wind + theta / 2)
         ] = np.sin(np.deg2rad(wind + theta / 2))
    plt.figure('after' + str(wind))
    plt.quiver(X, Y, Ucop, Vcop)
    plt.axis('equal')
    plt.axis([-120, 120, -120, 120])
    # plt.show()
    return Ucop, Vcop


def test0():
    plt.figure('test0')
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]
    U, V = rotation2(X, Y, 0, 0, R=3)
    print rotation2(1, 1, 0, 0, R=3)
    plt.quiver(X, Y, U, V, scale=100)
    plt.show()


if __name__ == '__main__':
    projectiontest(70)
    projectiontest(80)
    projectiontest(90)
    projectiontest(100)
    projectiontest(110)
    plt.show()
