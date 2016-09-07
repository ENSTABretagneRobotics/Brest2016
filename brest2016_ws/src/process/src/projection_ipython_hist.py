# coding: utf-8
import vectorFieldLib as vfl
import matplotlib.pyplot as plt
import numpy as np
from numpy import *

X, Y = np.mgrid[-100:100:40j, -100:100:40j]
U, V = vfl.patrouille_circulaire(X, Y, 0, 0, 1, 30, 5)
plt.quiver(X, Y, U, V)
U, V = vfl.patrouille_circulaire(X, Y, 0, 0, 1, 30, 10)
plt.quiver(X, Y, U, V)
U, V = vfl.patrouille_circulaire(X, Y, 0, 0, 1, 30, 20)
plt.quiver(X, Y, U, V)
plt.quiver(X, Y, U, V)
plt.axis([-120, 120, -120, 120])
plt.quiver(X, Y, U, V)
plt.axis([-120, 120, -120, 120])
U[0]
U[0, 0]
V[0, 0]
plt.quiver(X, Y, U, V)
plt.axis([-120, 120, -120, 120])
u00, v00 = U[0, 0], V[0, 0]
arctan(v00 / u00)
rad2deg(arctan(v00 / u00))
plt.quiver(X, Y, U, V)
plt.axis([-120, 120, -120, 120])
plt.axis('equal')
plt.quiver(X, Y, U, V)
plt.axis('equal')
plt.axis([-120, 120, -120, 120])
plt.quiver(X, Y, U, V, scale=100)
plt.axis('equal')
plt.axis([-120, 120, -120, 120])
plt.quiver(X, Y, U, V)
plt.axis('equal')
plt.axis([-120, 120, -120, 120])
T = arctan(V / U)
T[0, 0]
T = rad2deg(arctan(V / U))
T[0, 0]
wind = 45
Ttest = T.copy()
Ttest[wind - 22.5 < T < wind + 22.5] = nan
Ttest[wind - 22.5 < T < wind + 22.5] = nan
T < wind
T < wind + 22.5
wind - 22.5 < T < wind + 22.5
Ttest[(wind - 22.5) < T < (wind + 22.5)] = nan
Ttest[(wind - 22.5) <= T < (wind + 22.5)] = nan
Ttest[logical_and(wind - 22.5 < T, T < wind + 22.5)] = nan
Ttest
Utest = U.copy()
Vtest = V.copy()
Utest[logical_and(wind - 22.5 < T, T < wind + 22.5)] = 0
Vtest[logical_and(wind - 22.5 < T, T < wind + 22.5)] = 0
plt.quiver(X, Y, U, V)
plt.axis('equal')
plt.axis([-120, 120, -120, 120])
figure(2)
plt.quiver(X, Y, Utest, Vtest)
plt.axis('equal')
plt.axis([-120, 120, -120, 120])
figure(3)
Utest2 = U.copy()
Vtest2 = V.copy()
Utest[logical_and(wind - 22.5 < T, T < wind)] = wind - 22.5
Utest[logical_and(wind - 22.5 < T, T < wind + 22.5)] = 0
plt.quiver(X, Y, Utest, Vtest)
plt.axis('equal')
plt.axis([-120, 120, -120, 120])
arcsin(deg2rad(45))
arccos(deg2rad(45))
arccos(deg2rad(45))**2 + arcsin(deg2rad(45))
arccos(deg2rad(45))**2 + arcsin(deg2rad(45))**2
cos(deg2rad(45))
Utest2[logical_and(wind - 22.5 < T, T < wind)] = cos(deg2rad(wind))
Vtest2[logical_and(wind - 22.5 < T, T < wind)] = sin(deg2rad(wind))
Utest2[logical_and(wind - 22.5 < T, T < wind)] = cos(deg2rad(wind - 22.5))
Vtest2[logical_and(wind - 22.5 < T, T < wind)] = sin(deg2rad(wind - 22.5))
Utest2[logical_and(wind <= T, T < wind + 22.5)] = cos(deg2rad(wind + 22.5))
Vtest2[logical_and(wind <= T, T < wind + 22.5)] = sin(deg2rad(wind + 22.5))
plt.quiver(X, Y, Utest2, Vtest2)
plt.axis('equal')
plt.axis([-120, 120, -120, 120])
