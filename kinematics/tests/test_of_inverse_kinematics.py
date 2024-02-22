"""
Angles given in degrees
Joint angle ranges:
J1: (-90; 90)
J2: (-90, 90)
J3: (-180, 0)
"""

import pytest
import numpy as np
from numpy.testing import assert_allclose
from kinematics.kinematics_solvers import KinematicsSolver

ATOL = 1e-5
RTOL = 1e-5


def test_1():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([3, 0, 0])), np.array([0, 0, 0]), rtol=RTOL, atol=ATOL)

def test_2():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])

    angle1 = 24.35

    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([3*np.cos(np.deg2rad(angle1)), 3*np.sin(np.deg2rad(angle1)), 0])), np.array([angle1, 0, 0]), rtol=RTOL, atol=ATOL)

def test_3():

    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([0, 0, -1]))[1:], np.array([-90, -90]), rtol=RTOL, atol=ATOL)

def test_4():
    
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([np.sqrt(2), 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([0, 0, 0]))[1:], np.array([-90, -135]), rtol=RTOL, atol=ATOL)

def test_5():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([2, 0, 0])
    a3 = np.array([2*np.sqrt(2), 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([-1, 0, 0])), np.array([0, -90, -135]), rtol=RTOL, atol=ATOL)

def test_6():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([np.sqrt(2), 0, 0])
    a3 = np.array([np.sqrt(2), 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([3, 0, 0])), np.array([0, 45, -90]), rtol=RTOL, atol=ATOL)

def test_7():
    m1 = np.array([0, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([2, 0, 0])
    a3 = np.array([np.sqrt(5), 0, 0])
    leg = KinematicsSolver(m1, a1, a2, a3)
    assert_allclose(leg.inverse(np.array([-1, 0, -1])), np.array([0, -90, -180+np.rad2deg(np.arctan(2))]), rtol=RTOL, atol=ATOL)