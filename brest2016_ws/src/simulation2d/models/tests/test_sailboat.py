import matplotlib.pyplot as plt
from time import sleep
import numpy as np
import sys
sys.path.insert(
    0, '/home/ejalaa12/Desktop/Brest2016/brest2016_ws/src/simulation2d/models')
import sailboat

if __name__ == '__main__':
    awind = 2
    psi = np.pi / 2
    draw = True
    if draw:
        plt.ion()
    sb = sailboat.Sailboat(0., 0., 0., 1., 0.)
    u = np.array([0., 0.5])  # gouv, voile
    trajx = []
    trajy = []
    trajT = []
    for i in range(500):
        # u[0] = (np.random.random() - 0.5) / 10.
        u[0] = 0.5
        sb.simulate(u, awind, psi)
        trajx.append(sb.x)
        trajy.append(sb.y)
        trajT.append(sb.theta)
        if draw:
            # u[1] += 0.1
            plt.cla()
            # plt.hold()
            plt.axis('equal')
            plt.axis([-50, 50, -50, 50])
            sb.draw()
            sb.drawWind(awind, psi)
            plt.draw()
            sleep(0.01)
    plt.figure(1)
    plt.plot(trajx, trajy)
    plt.axis('equal')
    plt.figure(2)
    plt.plot(trajT)
    plt.axis('equal')
    plt.show()
