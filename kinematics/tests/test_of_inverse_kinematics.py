"""
Angles given in degrees
Joint angle ranges:
J1: (-90; 90)
J2: (-90, 90)
J3: (-180, 0)
"""

import numpy as np
from numpy.testing import assert_allclose
from kinematics.kinematics_solvers import KinematicsSolver
from . import ATOL, RTOL


def test_1():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([3, 0, 0])),
                    np.array([0, 0, 0]), rtol=RTOL, atol=ATOL)


def test_2():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    angle1 = 24.35

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([3*np.cos(np.deg2rad(angle1)),
                                          3*np.sin(np.deg2rad(angle1)), 0])),
                    np.array([angle1, 0, 0]), rtol=RTOL, atol=ATOL)


def test_3():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([0, 0, -1]))[1:],
                    np.array([-90, -90]), rtol=RTOL, atol=ATOL)


def test_4():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([np.sqrt(2), 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([0, 0, 0]))[1:],
                    np.array([-90, -135]), rtol=RTOL, atol=ATOL)


def test_5():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([2, 0, 0])
    a3 = np.array([2*np.sqrt(2), 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([-1, 0, 0])),
                    np.array([0, -90, -135]), rtol=RTOL, atol=ATOL)


def test_6():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([np.sqrt(2), 0, 0])
    a3 = np.array([np.sqrt(2), 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([3, 0, 0])),
                    np.array([0, 45, -90]), rtol=RTOL, atol=ATOL)


def test_7():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([2, 0, 0])
    a3 = np.array([np.sqrt(5), 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([-1, 0, -1])),
                    np.array([0, -90, -180+np.rad2deg(np.arctan(2))]),
                    rtol=RTOL, atol=ATOL)


def test_8():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([2, 0, 0])
    a3 = np.array([np.sqrt(5), 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([0, 1, -1])),
                    np.array([90, 0, -180+np.rad2deg(np.arctan(0.5))]),
                    rtol=RTOL, atol=ATOL)


def test_9():
    m1 = np.array([0, 0, 0.5])
    a1 = np.array([0.5, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)

    assert_allclose(leg.inverse(np.array([((0.5+np.sqrt(2)/2)*(np.sqrt(3)/2)),
                                          ((0.5+np.sqrt(2)/2)*0.5),
                                          (0.5+np.sqrt(2)/2 - 1)])),
                    np.array([30, 45, -135]), rtol=RTOL, atol=ATOL)


def test_10():
    m1 = np.array([0, 0, 0.5])
    a1 = np.array([0.5, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)

    assert_allclose(leg.inverse(np.array([((0.5)*(np.sqrt(3)/2)),
                                          ((0.5)*0.5), 0.5]))[[0, 2]],
                    np.array([30, -180]), rtol=RTOL, atol=ATOL)


def test_11():
    m1 = np.array([0, 0, 0.1])
    a1 = np.array([0.75, 0, 0])
    a2 = np.array([2, 0, 0])
    a3 = np.array([2, 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = -53.24
    assert_allclose(leg.inverse(np.array([0.75*np.cos(np.deg2rad(angle_1)),
                                          0.75*np.sin(np.deg2rad(angle_1)),
                                          0.1]))[[0, 2]],
                    np.array([angle_1, -180]), rtol=RTOL, atol=ATOL)


def test_12():
    m1 = np.array([0, 0, 0.2])
    a1 = np.array([0.4, 0, 0])
    a2 = np.array([1.2, 0, 0])
    a3 = np.array([2.3, 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([0.4, 0, 3.7])),
                    np.array([0, 90, 0]), rtol=RTOL, atol=ATOL)


def test_13():
    m1 = np.array([0, 0, 0.2])
    a1 = np.array([1.7, 0, 0])
    a2 = np.array([1.2, 0, 0])
    a3 = np.array([2.3, 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = 75.14
    assert_allclose(leg.inverse(np.array([1.7*np.cos(np.deg2rad(angle_1)),
                                          1.7*np.sin(np.deg2rad(angle_1)),
                                          3.7])),
                    np.array([angle_1, 90, 0]), rtol=RTOL, atol=ATOL)


def test_14():
    m1 = np.array([0, 0, 0.1])
    a1 = np.array([1, 0, 0])
    a2 = np.array([np.sqrt(2), 0, 0])
    a3 = np.array([1, 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([3, 0, 1.1])),
                    np.array([0, 45, -45]), rtol=RTOL, atol=ATOL)


def test_15():
    m1 = np.array([0, 0, 0.1])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([np.sqrt(2), 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([2, 0, 2.1])),
                    np.array([0, 90, -45]), rtol=RTOL, atol=ATOL)


def test_16():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([1, 0, -2])),
                    np.array([0, -90, 0]), rtol=RTOL, atol=ATOL)


def test_17():
    m1 = np.array([0, 0, 0.2])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1.4, 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([1, 0, -2.2])),
                    np.array([0, -90, 0]), rtol=RTOL, atol=ATOL)
