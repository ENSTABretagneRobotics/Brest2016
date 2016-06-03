#!usr/bin/env python
import numpy as np
import vectorFieldLib_v0 as vfl
import matplotlib.pyplot as plt


def draw_contour():
    """
    A------B
    |      |
    D------C
    """
    xa, ya = -100, 20
    xb, yb = 100, 20
    xc, yc = 100, -20
    xd, yd = -100, -20
    plt.plot([xa, xb, xc, xd, xa], [ya, yb, yc, yd, ya], 'b', linewidth=4)


def draw_obstacle():
    """
         K
    A---~/~---B
    |   ~/~   |
    D---~/~---C
         L
    """
    xk, yk = 0, 50
    xl, yl = 0, -50
    plt.plot([xk, xl], [yk, yl], 'r', linewidth=4)


def draw_ligne():
    X, Y = np.mgrid[-110:110:20j, -110:110:20j]
    U, V = vfl.ligne(
        X, Y, -100, 0, xb=100, yb=0, K=0.3, R=40, effect_range=100)
    plt.quiver(X, Y, U, V, scale=2)


def draw_security_wall():
    X, Y = np.mgrid[-110:110:20j, -110:110:20j]
    U, V = vfl.dir_segment(X, Y, -100, 40, 100, 40)
    plt.quiver(X, Y, U, V)


def draw_obstacle_avoidance():
    X, Y = np.mgrid[-110:110:20j, -110:110:20j]
    U, V = vfl.patrouille_circulaire(X, Y, 0, 0, K=0.3, R=50, turning_R=10)
    plt.quiver(X, Y, U, V)


def draw_ligne2():
    X, Y = np.mgrid[-110:110:20j, -110:110:20j]
    U, V = vfl.ligne(X, Y, 40, 0, xb=100, yb=0, K=5, R=40, effect_range=50)
    plt.quiver(X, Y, U, V)


def draw_part2():
    X, Y = np.mgrid[-110:110:20j, -110:110:20j]
    U0, V0 = vfl.ligne(X, Y, 40, 0, xb=100, yb=0, K=5, R=40, effect_range=50)
    U1, V1 = vfl.patrouille_circulaire(X, Y, 0, 0, K=1, R=50, turning_R=10)
    plt.quiver(X, Y, U0 + U1, V0 + V1)


if __name__ == '__main__':
    plt.figure('WRSC_MISSION')
    plt.hold(True)
    draw_contour()

    # Part 1
    # draw_ligne()
    # draw_security_wall()
    # Part 2
    draw_obstacle()
    # draw_obstacle_avoidance()
    # draw_ligne2()
    draw_part2()
    plt.show()
