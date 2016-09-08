from model import SimulationModel
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm, det

# Parameters
p1 = 0.1
p2 = 1.
p3 = 6000.
p4 = 1000.
p5 = 2000.
p6 = 1.
p7 = 1.
p8 = 2.
p9 = 300.
p10 = 10000.

# Environments variables
awind = 2   # force du vent
psi = np.pi / 4     # angle du vent
deltas = 0.
fs = 0.
fr = 0.


class Sailboat(SimulationModel):
    """Simulation d'un voilier
        Equation d'etat de L. Jaulin (cf ROBMOOC)
    """

    def __init__(self, x=0, y=0, theta=0, v=0, w=0):
        super(Sailboat, self).__init__()
        self.dt = 0.05
        # State
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.w = w
        self.X = np.array([x, y, theta, v, w])

        # command
        self.u = np.array([0, 0])
        # Drawing
        self.hull = np.array([[-1, 5, 7, 7, 5, -1, -1, -1],
                              [- 2, -2, -1, 1, 2, 2, -2, -2],
                              [1, 1, 1, 1, 1, 1, 1, 1]])
        self.sail = np.array([[-5, 0], [0, 0], [1, 1]])
        self.rudder = np.array([[-1, 1], [0, 0], [1, 1]])

    def draw(self):
        global fs, fr
        R = np.array([[np.cos(self.X[2]), -np.sin(self.X[2]), self.x],
                      [np.sin(self.X[2]), np.cos(self.X[2]), self.y],
                      [0, 0, 1]])

        # print R
        hull = np.dot(R, self.hull)

        Rdeltas = np.array([[np.cos(deltas), -np.sin(deltas), 3],
                            [np.sin(deltas), np.cos(deltas), 0],
                            [0, 0, 1]])
        Rdeltar = np.array([[np.cos(self.u[0]), -np.sin(self.u[0]), -1],
                            [np.sin(self.u[0]), np.cos(self.u[0]), 0],
                            [0, 0, 1]])

        sail = np.dot(np.dot(R, Rdeltas), self.sail)
        rudder = np.dot(np.dot(R, Rdeltar), self.rudder)

        Mfs = np.array([[-1., -1.], [0, -fs / 1000.], [1., 1.]])
        Mfr = np.array([[0., 0.], [0., fr / 100.], [1., 1.]])
        Mfs = np.dot(np.dot(R, Rdeltas), Mfs)
        Mfr = np.dot(np.dot(R, Rdeltar), Mfr)

        plt.plot(hull[0], hull[1], 'k', linewidth=2)
        plt.plot(sail[0], sail[1], 'b', linewidth=2)
        plt.plot(rudder[0], rudder[1], 'r', linewidth=2)
        plt.plot(Mfs[0], Mfs[1], 'g', linewidth=2)
        plt.plot(Mfr[0], Mfr[1], 'y', linewidth=2)

    def drawWind(self, coeff=1):
        windx = awind * np.cos(psi)
        windy = awind * np.sin(psi)
        windx *= coeff
        windy *= coeff
        plt.plot(10, 10, marker='o', markersize=5)
        plt.plot([10, 10 + windx], [10, 10 + windy])

    def fdot(self, u):
        global fs, fr, deltas
        self.u = u
        deltar = u[0]
        deltasmax = u[1]
        w_ap = np.array([awind * np.cos(psi - self.theta) - self.v,
                         awind * np.sin(psi - self.theta)])
        psi_ap = np.arctan2(w_ap[1], w_ap[0])
        a_ap = norm(w_ap)
        sigma = np.cos(psi_ap) + np.cos(deltasmax)
        if sigma < 0:
            deltas = np.pi + psi_ap
        else:
            deltas = -np.sign(np.sin(psi_ap)) * deltasmax
        fr = p5 * self.v * np.sin(deltar)
        fs = p4 * a_ap * np.sin(deltas - psi_ap)
        dx = self.v * np.cos(self.theta) + p1 * awind * np.cos(psi)
        dy = self.v * np.sin(self.theta) + p1 * awind * np.sin(psi)
        dtheta = self.w
        dv = (1. / p9) * (np.sin(deltas) * fs -
                          np.sin(deltar) * fr - p2 * self.v**2)
        dw = (1. / p10) * ((p6 - p7 * np.cos(deltas)) * fs -
                           p8 * np.cos(deltar) * fr -
                           p3 * self.w * self.v)
        Xdot = np.array([dx, dy, dtheta, dv, dw])
        return Xdot

    def control(self, vect):
        """
        Controleur pour suivre une commande en vecteur
        """
        zeta = np.pi / 4
        phi = np.arctan2(vect[1], vect[0])
        thetabar = phi
        if np.cos(psi - thetabar) + np.cos(zeta) < 0:
            thetabar = np.pi + psi - zeta
        deltar = (2 / np.pi) * np.arctan(np.tan(0.5 * (self.theta - thetabar)))
        deltasmax = np.pi / 4 * (np.cos(psi - thetabar) + 1)
        u = np.array([deltar, deltasmax])
        return u

    def simulate(self, u):
        # sim
        xdot = self.fdot(u)
        # print xdot
        self.X = self.X + xdot * self.dt
        self.x, self.y, self.theta, self.v, self.w = self.X
