from model import SimulationModel
import numpy as np
import matplotlib.pyplot as plt


class Char(SimulationModel):
    """Simulation d'un char"""

    def __init__(self, x=0, y=0, theta=0):
        super(Char, self).__init__()
        self.dt = 0.1
        self.x = x
        self.y = y
        self.theta = theta
        self.X = np.array([x, y, theta])
        self.img = np.array([[1, -1, 0, 0, -1, -1, 0, 0, -1, 1, 0, 0, 3, 3, 0],
                             [-2, -2, -2, -1, -1, 1, 1, 2, 2, 2, 2, 1, 0.5, -0.5, -1]])
        self.img = np.vstack(
            (self.img, np.ones(self.img.shape[1])))

    def draw(self):
        R = np.array([[np.cos(self.X[2]), -np.sin(self.X[2]), self.X[0]],
                      [np.sin(self.X[2]), np.cos(self.X[2]), self.X[1]],
                      [0, 0, 1]])

        self.img = np.dot(R, self.img)
        plt.plot(self.img[0], self.img[1])

    def fdot(self, u):
        xdot = u[1] * np.cos(self.X[2])
        ydot = u[1] * np.sin(self.X[2])
        thetadot = u[0]
        return np.array([xdot, ydot, thetadot])

    def simulate(self, u):
        self.X = self.X + self.fdot(u) * self.dt
        self.x, self.y, self.theta = self.X
