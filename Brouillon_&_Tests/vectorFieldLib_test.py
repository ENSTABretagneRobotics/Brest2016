#!usr/bin/env python
import numpy as np
import unittest
import vectorFieldLib_v0 as vfl
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

    def test_ligne(self):
        vfl.ligne(0, 0, 0, 0, 1, 1, 20)

    def test_tournant(self):
        np.testing.assert_array_equal(
            vfl.dir_tournant(0, 1), np.array([-1, 0]))


class ObjectivesTestPlot():
    def test_dir_point(self):
        plt.figure('dir_point (5,5)')
        X, Y = np.mgrid[-20:20:20j, -20:20:20j]
        U, V = vfl.dir_point(X, Y, 5, 5)
        plt.quiver(X, Y, U, V)
        plt.show()
        # plt.draw()

    def test_dir_tournant(self):
        plt.figure('dir_tournant (-5,-5)')
        X, Y = np.mgrid[-20:20:10j, -20:20:10j]
        U, V = vfl.dir_tournant(X, Y, 0, 0)
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

    def test_dir_segment_complet(self):
        plt.figure('test segment complet')
        X, Y = np.mgrid[-20:20:20j, -20:20:20j]
        U1, V1 = vfl.dir_segment(X, Y, -5, 0, 5, 0)
        U2, V2 = vfl.dir_segment_extremity(X, Y, -5, 0, 5, 0)
        plt.quiver(X, Y, U1 + U2, V1 + V2)
        plt.show()

    def test_obstacle_point(self):
        plt.figure('obstacle (0,0), K=3, R=3, security=MEDIUM')
        X, Y = np.mgrid[-100:100:40j, -100:100:40j]
        U, V = vfl.obstacle_point(
            X, Y, 0, 0, K=0.2, R=20, security='MEDIUM',
            slowing_R=1, slowing_K=3)
        x, y = vfl.obstacle_point(
            3, 4, 0, 0, K=0.2, R=20, security='MEDIUM',
            slowing_R=1, slowing_K=3)
        print x, y
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
            X, Y, 0, 0, xb=50, yb=0, K=0.2, R=5, effect_range=20)
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

    def test_patrouille_circulaire(self):
        plt.figure('patrouille circle 0,5')
        X, Y = np.mgrid[-20:20:40j, -20:20:40j]
        U, V = vfl.patrouille_circulaire(X, Y, 0, 5, K=2, R=5)
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

    def exemple_patrouille(self):
        "Composition d'un champ de patrouille"
        X, Y = np.mgrid[-20:20:20j, -20:20:20j]
        t = np.linspace(-20, 20, 100)
        d = np.vectorize(vfl.dist_point)(X, Y, 0, 0)
        R = 7
        L = 2

        plt.figure(
            """Champ direction oppose centre du cercle
            (a noter qu'on met juste -1)""")
        Uc, Vc = -vfl.dir_point(X, Y, 0, 0)
        plt.hold(True)
        plt.quiver(X, Y, Uc, Vc)
        pc = vfl.gaussienne(t, R, 0) - 0.5
        plt.axis('equal')
        # plt.plot(t, pc)

        plt.figure("Champ tournant")
        Ut, Vt = vfl.dir_tournant(X, Y)
        plt.hold(True)
        plt.quiver(X, Y, Ut, Vt)
        pt = vfl.gaussienne(t, L, R) + vfl.gaussienne(t, 2, -R)
        plt.axis('equal')
        # plt.plot(t, pt)

        plt.figure("Profil champ tournant au cercle")
        plt.plot(t, pt)
        plt.plot([0, 0], [0, 1], 'k', linewidth=2)
        plt.axis([-20, 20, 0, 1])
        plt.grid(True)

        plt.figure("Profil champ cercle attractif")
        plt.plot(t, pc)
        plt.plot(t, 0 * pc, 'k', linewidth=2)
        plt.plot([0, 0], [-0.6, 0.6], 'k', linewidth=2)
        plt.axis([-20, 20, -0.6, 0.6])
        plt.grid(True)

        # Composition 1
        plt.figure("Compo 1")
        f = vfl.gaussienne(d, R, 0) - 0.5
        Uc, Vc = np.array([Uc, Vc]) * f
        plt.quiver(X, Y, Uc, Vc)
        plt.axis('equal')

        # Composition 2
        plt.figure("Compo 2")
        bosse = vfl.gaussienne(d, L, R)
        Ut, Vt = np.array([Ut, Vt]) * bosse
        plt.quiver(X, Y, Ut, Vt)
        plt.axis('equal')

        # Composition Finale
        plt.figure("Compo Finale")
        plt.quiver(X, Y, Uc + Ut, Vc + Vt)
        plt.axis('equal')

        plt.show()

if __name__ == '__main__':
    # plots
    otp = ObjectivesTestPlot()
    # otp.test_dir_point()
    # otp.test_dir_tournant()
    # otp.test_dir_segment_complet()
    # plot_test.test_parameter()
    # otp.exemple_patrouille()
    # otp.test_obstacle_point()
    otp.test_ligne()
    otp.test_patrouille_circulaire()

    # run all ObjectivesTestPlot methods
    # print dir(plot_test)
    # for name, method in ObjectivesTestPlot.__dict__.iteritems():
    #     if callable(method) and name not in ['onclick_calc', 'test_parameter']:
    #         method(None)
    # unittest
    # unittest.main()
