"""
Angles given in degrees
Joint angle ranges:
J1: (-90; 90)
J2: (-90, 90)
J3: (-180, 0)
"""

import numpy as np
from numpy.testing import assert_allclose
from ..kinematics_solvers import KinematicsSolver
from . import ATOL, RTOL


def test_1():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.forward(np.array([0, 0, 0])),
                    np.array([3, 0, 0]), rtol=RTOL, atol=ATOL)


def test_2():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.forward(np.array([0, -90, -90])),
                    np.array([0, 0, -1]), rtol=RTOL, atol=ATOL)

    m1 = np.array([0, 0, 0.17])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = 12.24
    assert_allclose(leg.forward(np.array([angle_1, 0, 0])),
                    np.array([3*np.cos(np.deg2rad(angle_1)),
                              3*np.sin(np.deg2rad(angle_1)), 0.17]),
                    rtol=RTOL, atol=ATOL)


def test_4():
    m1 = np.array([0, 0, 0.1])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = -84.52
    assert_allclose(leg.forward(np.array([angle_1, 0, 0])),
                    np.array([3*np.cos(np.deg2rad(angle_1)),
                              3*np.sin(np.deg2rad(angle_1)), 0.1]),
                    rtol=RTOL, atol=ATOL)


def test_5():
    m1 = np.array([0, 0, -1.33])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.forward(np.array([0, 60, -30])),
                    np.array([1+np.cos(np.pi/3) + np.cos(np.pi/6), 0,
                              np.sin(np.pi/3)+np.sin(np.pi/6)-1.33]),
                    rtol=RTOL, atol=ATOL)


def test_6():
    m1 = np.array([0, 0, -5.41])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = 34.12
    angle_2 = -23.11
    assert_allclose(leg.forward(np.array([angle_1, angle_2, -180])),
                    np.array([np.cos(np.deg2rad(angle_1)),
                              np.sin(np.deg2rad(angle_1)), -5.41]),
                    rtol=RTOL, atol=ATOL)


def test_7():
    m1 = np.array([0, 0, -1])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = -77.81
    angle_2 = 42.11
    assert_allclose(leg.forward(np.array([angle_1, angle_2, -180])),
                    np.array([np.cos(np.deg2rad(angle_1)),
                              np.sin(np.deg2rad(angle_1)), -1]),
                    rtol=RTOL, atol=ATOL)


def test_8():
    m1 = np.array([0, 0, 0.92])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = -21.21
    angle_2 = -32.14
    angle_3 = -81.23

    R = 1 + np.sin(np.deg2rad(90+angle_2)) - \
        np.cos(np.deg2rad(180+angle_2+angle_3))
    X = np.cos(np.deg2rad(angle_1))*R
    Y = np.sin(np.deg2rad(angle_1))*R
    Z = 0.92 - np.cos(np.deg2rad(90+angle_2)) - \
        np.sin(np.deg2rad(180+angle_2+angle_3))

    assert_allclose(leg.forward(np.array([angle_1, angle_2, angle_3])),
                    np.array([X, Y, Z]), rtol=RTOL, atol=ATOL)


def test_9():
    m1 = np.array([0, 0, 0.32])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = 30.29
    angle_2 = 52.43
    angle_3 = -164.13

    R = 1 + np.cos(np.deg2rad(angle_2)) - \
        np.sin(np.deg2rad(90-angle_2-(180+angle_3)))
    X = np.cos(np.deg2rad(angle_1))*R
    Y = np.sin(np.deg2rad(angle_1))*R
    Z = 0.32 + np.sin(np.deg2rad(angle_2)) - \
        np.cos(np.deg2rad(90-angle_2-(180+angle_3)))

    assert_allclose(leg.forward(np.array([angle_1, angle_2, angle_3])),
                    np.array([X, Y, Z]), rtol=RTOL, atol=ATOL)


def test_10():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([np.sqrt(2), 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)

    angle_1 = 68.21
    assert_allclose(leg.forward(np.array([angle_1, -90, -135])),
                    m1, rtol=RTOL, atol=ATOL)


def test_11():
    m1 = np.array([0, 0, 4.24])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([np.sqrt(2), 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)

    angle_1 = -76.11
    assert_allclose(leg.forward(np.array([angle_1, -90, -135])),
                    m1, rtol=RTOL, atol=ATOL)


def test_12():
    m1 = np.array([0, 0, 0])
    a1 = np.array([2.5, 0, 0])
    a2 = np.array([3, 0, 0])
    a3 = np.array([3.6, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)

    assert_allclose(leg.forward(np.array([0, 0, 0])),
                    np.array([9.1, 0, 0]), rtol=RTOL, atol=ATOL)
