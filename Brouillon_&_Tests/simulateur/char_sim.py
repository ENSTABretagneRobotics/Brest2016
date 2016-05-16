import matplotlib.pyplot as plt
import numpy as np


# def draw_tank(x, y, theta):
#     M = np.array([[1, -1, 0, 0, -1, -1, 0, 0, -1, 1, 0, 0, 3, 3, 0],
#                   [-2, -2, -2, -1, -1, 1, 1, 2, 2, 2, 2, 1, 0.5, -0.5, -1]])
#     M = np.vstack((M, np.ones(M.shape[1])))
#     R = np.array([[np.cos(theta), -np.sin(theta), x],
#                   [np.sin(theta), np.cos(theta), y],
#                   [0, 0, 1]])

#     M = np.dot(R, M)
#     plt.plot(M[0], M[1])


def draw_tank(x):
    M = np.array([[1, -1, 0, 0, -1, -1, 0, 0, -1, 1, 0, 0, 3, 3, 0],
                  [-2, -2, -2, -1, -1, 1, 1, 2, 2, 2, 2, 1, 0.5, -0.5, -1]])
    M = np.vstack((M, np.ones(M.shape[1])))
    R = np.array([[np.cos(x[2]), -np.sin(x[2]), x[0]],
                  [np.sin(x[2]), np.cos(x[2]), x[1]],
                  [0, 0, 1]])

    M = np.dot(R, M)
    plt.plot(M[0], M[1])


def fdot(x, u):
    xdot = u[1] * np.cos(x[2])
    ydot = u[1] * np.sin(x[2])
    thetadot = u[0]
    return np.array([xdot, ydot, thetadot])


if __name__ == '__main__':
    plt.ion()
    x = np.array([-20, -10, 4])
    dt = 0.1
    u = [0, 1]
    for t in np.arange(1, 30, dt):
        plt.cla()
        x = x + fdot(x, u) * dt
        draw_tank(x)
        plt.axis([-30, 30, -30, 30])
        plt.draw()
