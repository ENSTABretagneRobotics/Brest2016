from fields import *
import matplotlib.pyplot as plt
import numpy as np
from time import sleep


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


def angnn(ang):
    while ang <= -180:
        ang += 360
    while ang > 180:
        ang -= 360
    return ang


def inNoGoZone(wind, theta, angle):
    wind = np.radians(wind)
    theta = np.radians(theta)
    angle = np.radians(angle)
    return np.logical_not(np.cos(angle - wind) > np.cos(wind - theta / 2))


def sideNoGoZone(wind, angle):
    wind = np.radians(wind)
    angle = np.radians(angle)
    # 1 is left
    return np.sign(np.sin(angle - wind)) == 1


def projectiontest(wind):
    X, Y = np.mgrid[-100:100:30j, -100:100:30j]
    # U, V = patrouille_circulaire(X, Y, 0, 0, 1, 30, 20)
    U, V = champ_constant(X, Y, 0, 0)
    # U, V = waypoint(X, Y, 0, 0)
    # U, V = limite(X, Y, -40, 0, 40, 0, K=1, R=3, security='LOW')

    # U1, V1 = limite(X, Y, -40, 0, 40, 0, K=5, R=10, security='LOW')
    # U2, V2 = waypoint(X, Y, 70, 70)
    # U, V = U1 + U2, V1 + V2
    # U, V = ligne(X, Y, -60, -0, 60, 0, K=1, R=5, effect_range=100)
    # plt.figure('before')
    # plt.quiver(X, Y, U, V)
    # plt.axis('equal')
    # plt.axis([-120, 120, -120, 120])

    # Wind and opening angle
    wind = angnn(wind)  # todo correct the projection to avoid this tweak
    theta = 100
    # angle value of the vector field
    # T = np.rad2deg(np.arctan2(V, U))
    T = np.rad2deg(np.arctan2(V, U))
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

    # inside = inNoGoZone(wind, theta, T)
    # left = sideNoGoZone(wind, T)
    # right = np.logical_not(left)

    Ucop = U.copy()
    Vcop = V.copy()
    # --------------------------------------------------------------------------
    Ucop[left] = np.cos(np.deg2rad(wind - theta / 2))
    Vcop[left] = np.sin(np.deg2rad(wind - theta / 2))
    Ucop[right] = np.cos(np.deg2rad(wind + theta / 2))
    Vcop[right] = np.sin(np.deg2rad(wind + theta / 2))
    # --------------------------------------------------------------------------
    # Ucop[np.logical_and(inside, left)] = -np.cos(np.deg2rad(wind - theta / 2))
    # Vcop[np.logical_and(inside, left)] = -np.sin(np.deg2rad(wind - theta / 2))

    # Ucop[np.logical_and(inside, right)] = -np.cos(np.deg2rad(wind + theta / 2))
    # Vcop[np.logical_and(inside, right)] = -np.sin(np.deg2rad(wind + theta / 2))
    # --------------------------------------------------------------------------
    # Ucop[np.logical_and(inNoGoZone(wind, theta, T))
    #      ] = np.cos(np.deg2rad(wind - theta / 2))
    # Vcop[np.logical_and(low < T, T < mid)
    #      ] = np.sin(np.deg2rad(wind - theta / 2))

    # Ucop[np.logical_and(mid <= T, T < upp)
    #      ] = np.cos(np.deg2rad(wind + theta / 2))
    # Vcop[np.logical_and(mid <= T, T < upp)
    #      ] = np.sin(np.deg2rad(wind + theta / 2))

    # print T
    # print np.logical_and(low < T, T < mid)
    # print np.logical_and(mid <= T, T < upp)
    # print '\n' * 4
    # # print U
    # # print V
    # # print Ucop
    # # print Vcop
    # # plt.figure('after')
    plt.quiver(X, Y, Ucop, Vcop)
    plt.axis('equal')
    plt.axis([-120, 120, -120, 120])
    # # plt.show()
    # return Ucop, Vcop


def test0():
    plt.figure('test0')
    X, Y = np.mgrid[-20:20:4j, -20:20:4j]
    U, V = rotation2(X, Y, 0, 0, R=3)
    print rotation2(1, 1, 0, 0, R=3)
    plt.quiver(X, Y, U, V, scale=100)
    plt.show()


if __name__ == '__main__':
    plt.ion()
    # for i in xrange(-180, 180):
    for i in xrange(44, 46):
        print i
        X, Y = np.mgrid[-100:100:10j, -100:100:3j]
        # U, V = patrouille_circulaire(X, Y, 0, 0, 1, 30, 20)
        U, V = waypoint(X, Y, 40, 0)
        Ucop, Vcop = projection(U, V, i, 100)
        plt.figure('after')
        plt.cla()
        plt.quiver(X, Y, Ucop, Vcop)
        plt.axis('equal')
        plt.axis([-120, 120, -120, 120])
        # projectiontest(i)
        plt.draw()
        # sleep(1)
    # projectiontest(180)
    # plt.show()
