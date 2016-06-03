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


def draw_ligne():
    X, Y = np.mgrid[-110:110:20j, -110:110:20j]
    U, V = vfl.ligne(
        X, Y, -100, 0, xb=100, yb=0, K=0.3, R=40, effect_range=100)
    plt.quiver(X, Y, U, V, scale=2)


def draw_security_wall():
    X, Y = np.mgrid[-110:110:20j, -110:110:20j]
    U, V = vfl.dir_segment(X, Y, -100, 40, 100, 40)
    plt.quiver(X, Y, U, V)


if __name__ == '__main__':
    plt.figure('WRSC_MISSION')
    plt.hold(True)
    draw_contour()
    draw_ligne()
    # draw_security_wall()
    plt.show()
