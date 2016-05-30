#!usr/bin/env python
import numpy as np
import unittest
import vectorFieldLib as vfl
import matplotlib.pyplot as plt


class TestBasicFunctions(unittest.TestCase):
    """Testing basic function of the library for vector fields"""

    def test_translate(self):
        self.assertEqual(vfl.translate(0, 0, 1, 1), [-1, -1])

    def test_rotate(self):
        np.testing.assert_array_equal(vfl.rotate(0, 0, np.pi), [0, 0])
        np.testing.assert_allclose(vfl.rotate(0, 1, np.pi),
                                   [0, -1],
                                   rtol=0, atol=1e-15)

    def test_normalize(self):
        np.testing.assert_array_equal(
            vfl.normalize(np.array([0, 2])), np.array([0, 1]))

    def test_dist_point(self):
        self.assertEqual(vfl.dist_point(0, 0, 5, 5), np.sqrt(50))

    def test_dist_droite(self):
        self.assertAlmostEqual(vfl.dist_droite(5, 0, 0, -10, 0, 10), -5)

    def test_dir_point(self):
        plt.figure('dir_point (5,5)')
        X, Y = np.mgrid[-20:20:40j, -20:20:40j]
        U, V = vfl.dir_point(X, Y, 5, 5)
        plt.quiver(X, Y, U, V)
        plt.show()
        # plt.draw()

    def test_dir_tournant(self):
        plt.figure('dir_tournant (-5,-5)')
        X, Y = np.mgrid[-20:20:40j, -20:20:40j]
        U, V = vfl.dir_tournant(X, Y, -5, -5)
        plt.quiver(X, Y, U, V)
        plt.show()
        # todo: le champ tournant n'est pas de norme constante

    def test_dirac(self):
        pass

    def test_zone_segment(self):
        self.assertEqual(vfl.zone_segment(-1, 1, 0, 0, 20, 0), 'EX_L')
        self.assertEqual(vfl.zone_segment(-1, 0, 0, 0, 20, 0), 'EX_L')
        self.assertEqual(vfl.zone_segment(-1, -1, 0, 0, 20, 0), 'EX_L')
        self.assertEqual(vfl.zone_segment(0, -1, 0, 0, 20, 0), 'IN_R')
        self.assertEqual(vfl.zone_segment(10, -1, 0, 0, 20, 0), 'IN_R')
        self.assertEqual(vfl.zone_segment(20, -1, 0, 0, 20, 0), 'IN_R')
        self.assertEqual(vfl.zone_segment(23, -1, 0, 0, 20, 0), 'EX_R')
        self.assertEqual(vfl.zone_segment(23, 0, 0, 0, 20, 0), 'EX_R')
        self.assertEqual(vfl.zone_segment(23, 1, 0, 0, 20, 0), 'EX_R')
        self.assertEqual(vfl.zone_segment(20, 1, 0, 0, 20, 0), 'IN_L')
        self.assertEqual(vfl.zone_segment(10, 1, 0, 0, 20, 0), 'IN_L')
        self.assertEqual(vfl.zone_segment(0, 1, 0, 0, 20, 0), 'IN_L')
        # Extremites segment
        self.assertEqual(vfl.zone_segment(0, 0, 0, 0, 20, 0), 'IN_L')
        self.assertEqual(vfl.zone_segment(20, 0, 0, 0, 20, 0), 'IN_L')
        # Sur segment
        self.assertEqual(vfl.zone_segment(10, 0, 0, 0, 20, 0), 'IN_L')

    def test_champ_constant(self):
        plt.figure('champ constant (-5,-5)')
        X, Y = np.mgrid[-20:20:40j, -20:20:40j]
        U, V = vfl.champ_constant(X, Y, -5, -5)
        plt.quiver(X, Y, U, V)
        plt.show()

    def test_waypoint(self):
        plt.figure('waypoint (0,5)')
        X, Y = np.mgrid[-20:20:40j, -20:20:40j]
        U, V = vfl.waypoint(X, Y, 0, 5)
        plt.quiver(X, Y, U, V)
        plt.show()

    def test_obstacle_point(self):
        plt.figure('obstacle (0,0), K=3, R=3, security=MEDIUM')
        X, Y = np.mgrid[-100:100:40j, -100:100:40j]
        U, V = vfl.obstacle_point(
            X, Y, 0, 0, K=0.2, R=20, security='MEDIUM',
            slowing_R=1, slowing_K=3)
        plt.quiver(X, Y, U, V, scale=2)
        plt.show()

    def test_champ_compose(self):
        " Champ compose d'un waypoint et d'un obstacle MEDIUM"
        plt.figure('Waypoint(0,5), Obstacle(0,0,K=3,R=3)')
        X, Y = np.mgrid[-10:10:40j, -10:10:40j]
        Uo, Vo = vfl.obstacle_point(
            X, Y, 0, 0, K=3, R=3,
            security='HIGH', slowing_R=5, slowing_K=5)
        print Uo, Vo
        Uw, Vw = vfl.waypoint(X, Y, 0, 5)
        U, V = Uo + Uw, Vo + Vw
        plt.quiver(X, Y, U, V)
        plt.axis('equal')
        plt.show()


def onclick_calc(event=None):
    x = float(event.xdata)
    y = float(event.ydata)
    print vfl.zone_segment(x, y, 0, 0, 10, 0)


def test_parameter():
    # print ligne(1, 1, 0, 0, 10, 0, s=-1, r=1)
    fig = plt.figure('total')
    plt.plot([0, 10], [0, 0])
    fig.canvas.mpl_connect('button_press_event', onclick_calc)
    plt.axis([-20, 20, -20, 20])
    plt.show()

if __name__ == '__main__':
    unittest.main()
    # test_parameter()
