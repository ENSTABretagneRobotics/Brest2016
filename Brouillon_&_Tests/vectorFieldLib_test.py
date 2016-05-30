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

    def test_securities(self):
        vfl.profil_security_M(0, 0, 10, 10, 3, 5, security='HIGH',
                              slowing_R=0.5, slowing_K=10)
        vfl.profil_security_M(0, 0, 10, 10, 3, 5, security='MEDIUM',
                              slowing_R=0.5, slowing_K=10)
        vfl.profil_security_M(0, 0, 10, 10, 3, 5, security='LOW',
                              slowing_R=0.5, slowing_K=10)


class ObjectivesTestPlot():
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

    def test_dir_segment(self):
        plt.figure('test_dir_segment')
        X, Y = np.mgrid[-20:20:40j, -20:20:40j]
        U, V = vfl.dir_segment(X, Y, -5, 0, 10, 0)
        plt.quiver(X, Y, U, V)
        plt.show()

    def test_dir_segment_extrimity(self):
        plt.figure('test_dir_segment_extrimity')
        X, Y = np.mgrid[-20:20:40j, -20:20:40j]
        U, V = vfl.dir_segment_extremity(X, Y, -5, 0, 10, 0)
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

    def test_limite(self):
        plt.figure('limite security=MEDIUM')
        X, Y = np.mgrid[-100:100:40j, -100:100:40j]
        U, V = vfl.limite(
            X, Y, 0, 0, xb=50, yb=0, K=0.2, R=20, security='MEDIUM',
            slowing_R=5, slowing_K=3)
        plt.quiver(X, Y, U, V, scale=5)
        plt.show()

    def test_ligne(self):
        plt.figure('LIGNE ATTRACTIVE')
        X, Y = np.mgrid[-100:100:40j, -100:100:40j]
        U, V = vfl.ligne(
            X, Y, 0, 0, xb=50, yb=0, K=0.2, R=20)
        plt.quiver(X, Y, U, V, scale=2)
        plt.show()

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

    def test_waypoint_medium_obstacle(self):
        " Champ compose d'un waypoint et d'un obstacle MEDIUM"
        plt.figure('Waypoint(0,5), MEDIUM Obstacle(0,0,K=2,R=5)')
        X, Y = np.mgrid[-10:10:40j, -10:10:40j]
        Uo, Vo = vfl.obstacle_point(
            X, Y, 0, 0, K=2, R=5, security='MEDIUM',
            slowing_R=1, slowing_K=2)
        Uw, Vw = vfl.waypoint(X, Y, 0, 5)
        U, V = Uo + Uw, Vo + Vw
        plt.quiver(X, Y, U, V)
        plt.axis('equal')
        plt.show()

    def test_waypoint_medium_limite(self):
        " Champ compose d'un waypoint et d'un obstacle MEDIUM"
        plt.figure('Waypoint(0,5), MEDIUM Obstacle(0,0,K=2,R=5)')
        X, Y = np.mgrid[-100:100:40j, -100:100:40j]
        Uo, Vo = vfl.limite(
            X, Y, 0, 0, xb=50, yb=0, K=2, R=20, security='MEDIUM',
            slowing_R=3, slowing_K=5)
        Uw, Vw = vfl.waypoint(X, Y, 0, 50)
        U, V = Uo + Uw, Vo + Vw
        plt.quiver(X, Y, U, V)
        plt.axis('equal')
        plt.show()

    def test_waypoint_high_obstacle(self):
        " Champ compose d'un waypoint et d'un obstacle HIGH"
        plt.figure('Waypoint(0,5), HIGH Obstacle(0,0,K=2,R=5)')
        X, Y = np.mgrid[-20:20:40j, -20:20:40j]
        Uo, Vo = vfl.obstacle_point(
            X, Y, 0, 0, K=2, R=5, security='HIGH')
        Uw, Vw = vfl.waypoint(X, Y, 0, 5)
        U, V = Uo + Uw, Vo + Vw
        plt.quiver(X, Y, U, V)
        plt.axis('equal')
        plt.show()

    def onclick_calc(self, event=None):
        x = float(event.xdata)
        y = float(event.ydata)
        print vfl.zone_segment(x, y, 0, 0, 10, 0)

    def test_parameter(self):
        # print ligne(1, 1, 0, 0, 10, 0, s=-1, r=1)
        fig = plt.figure('total')
        plt.plot([0, 10], [0, 0])
        fig.canvas.mpl_connect('button_press_event', self.onclick_calc)
        plt.axis([-20, 20, -20, 20])
        plt.show()

if __name__ == '__main__':
    # plots
    plot_test = ObjectivesTestPlot()
    # plot_test.test_parameter()
    # plot_test.test_dir_segment()
    # plot_test.test_dir_segment_extrimity()
    # plot_test.test_limite()
    # plot_test.test_ligne()
    # plot_test.test_waypoint_medium_limite()

    # run all ObjectivesTestPlot methods
    # print dir(plot_test)
    for name, method in ObjectivesTestPlot.__dict__.iteritems():
        if callable(method) and name not in ['onclick_calc', 'test_parameter']:
            method(None)
    # unittest
    unittest.main()
