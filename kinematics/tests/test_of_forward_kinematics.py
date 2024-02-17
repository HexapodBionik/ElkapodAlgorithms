"""
Kąty są w stopniach.
Zakresy serw zgodnie z tym co omawiano:
J1: -90, 90
J2: -90, 90
J3: -180, 0
"""

import pytest
import numpy as np
from numpy.testing import assert_allclose
from kinematics.kinematics_solvers import KinematicsSolver

ATOL = 1e-8
RTOL = 1e-8

DEG_TO_RAD = np.pi/180

def test_1():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.forward(np.array([0, 0, 0])), np.array([3, 0, 0]), rtol=RTOL, atol=ATOL)

def test_2():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.forward(np.array([0, -90, -90])), np.array([0, 0, -1]), rtol=RTOL, atol=ATOL)

def test_3():
    m1 = np.array([1.2, -0.5, 0.17])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = 12.24
    assert_allclose(leg.forward(np.array([angle_1, 0, 0])), np.array([3*np.cos(angle_1*DEG_TO_RAD)+1.2, 3*np.sin(angle_1*DEG_TO_RAD)-0.5, 0.17]), rtol=RTOL, atol=ATOL)

def test_4():
    m1 = np.array([1.4, -2.5, 0.1])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = -84.52
    assert_allclose(leg.forward(np.array([angle_1, 0, 0])), np.array([3*np.cos(angle_1*DEG_TO_RAD)+1.4, 3*np.sin(angle_1*DEG_TO_RAD)-2.5, 0.1]), rtol=RTOL, atol=ATOL)

def test_5():
    m1 = np.array([0.9, 0.1, -1.33])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.forward(np.array([0, 60, -30])), np.array([1+np.sqrt(3)/2 + 0.9, 0.1, 1.5-1.33]), rtol=RTOL, atol=ATOL)

def test_5():
    m1 = np.array([0.1, 2.3, -5.41])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = 34.12
    angle_2 = -23.11
    assert_allclose(leg.forward(np.array([angle_1, angle_2, -180])), np.array([np.cos(angle_1*DEG_TO_RAD)+0.1, np.sin(angle_1*DEG_TO_RAD)+2.3, -5.41]), rtol=RTOL, atol=ATOL)

def test_6():
    m1 = np.array([1.1, 0.4, -1])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = -77.81
    angle_2 = 42.11
    assert_allclose(leg.forward(np.array([angle_1, angle_2, -180])), np.array([np.cos(angle_1*DEG_TO_RAD)+1.1, np.sin(angle_1*DEG_TO_RAD)+0.4, -1]), rtol=RTOL, atol=ATOL)

def test_7():
    m1 = np.array([2.01, -1.24, 0.92])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = -21.21
    angle_2 = -32.14
    angle_3 = -81.23
    
    R = 1 + np.sin((90+angle_2)*DEG_TO_RAD) - np.cos((180+angle_2+angle_3)*DEG_TO_RAD)
    X = 2.01 + np.cos(angle_1*DEG_TO_RAD)*R
    Y = (-1.24) + np.sin(angle_1*DEG_TO_RAD)*R
    Z = 0.92 + np.cos((90+angle_2)*DEG_TO_RAD) + np.sin((180+angle_2+angle_3)*DEG_TO_RAD)

    assert_allclose(leg.forward(np.array([angle_1, angle_2, angle_3])), np.array([X, Y, Z]))

def test_8():
    m1 = np.array([3.14, 1.59, 0.32])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    angle_1 = 30.29
    angle_2 = 52.43
    angle_3 = -164.13
    
    R = 1 + np.cos(angle_2*DEG_TO_RAD) - np.sin((90-angle_2-(180+angle_3))*DEG_TO_RAD)
    X = 3.14 + np.cos(angle_1*DEG_TO_RAD)*R
    Y = 1.59 + np.sin(angle_1*DEG_TO_RAD)*R
    Z = 0.32 + np.sin(angle_2*DEG_TO_RAD) - np.cos((90-angle_2-(180+angle_3))*DEG_TO_RAD)
    
    assert_allclose(leg.forward(np.array([angle_1, angle_2, angle_3])), np.array([X, Y, Z]))

def test_9():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([np.sqrt(2), 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)

    angle_1 = 68.21
    assert_allclose(leg.forward(np.array([angle_1, -90, -135])), m1)

def test_10():
    m1 = np.array([0.25, -1.32, 4.24])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([np.sqrt(2), 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)

    angle_1 = -76.11
    assert_allclose(leg.forward(np.array([angle_1, -90, -135])), m1)

def test_11():
    m1 = np.array([0.5, 0.1, 0])
    a1 = np.array([2.5, 0, 0])
    a2 = np.array([3, 0, 0])
    a3 = np.array([3.6, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)

    assert_allclose(leg.forward(np.array([0,0,0])), np.array([9.6, 0.1, 0]))

