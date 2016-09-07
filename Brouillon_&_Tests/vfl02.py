from vectorFieldLib_v0 import *
import matplotlib.pyplot as plt
import numpy as np


def rotation2(x, y, a, b, K=1, R=1):
    """
    Genere un cercle attractif tournant (a gauche)
    autour du point(a,b) et de rayon R
    """
    # profil 2
    # x, y = translate(x, y, a, b)
    d = np.vectorize(dist_point)(x, y, a, b)
    rot = dir_tournant(x, y, a, b)
    # print d
    # print abs(d) < R
    # print rot[0]
    rot[0][abs(d) > R] = 0
    rot[1][abs(d) > R] = 0
    return K * rot


def test0():
    plt.figure('test0')
    X, Y = np.mgrid[-20:20:40j, -20:20:40j]
    U, V = rotation2(X, Y, 0, 0, R=3)
    plt.quiver(X, Y, U, V, scale=100)
    plt.show()


if __name__ == '__main__':
    test0()
