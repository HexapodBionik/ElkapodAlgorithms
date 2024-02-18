from pytest import raises
from kinematics.kinematics_solvers import KinematicsSolver
import numpy as np
from numpy.testing import assert_allclose

ATOL = 1e-8
RTOL = 1e-8


def test_kinematics_proper_initialization():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, -0.035])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    kinematics_solver = KinematicsSolver(m1, a1, a2, a3)

    assert_allclose(m1, kinematics_solver.m1, rtol=RTOL, atol=ATOL)
    assert_allclose(a1, kinematics_solver.a1, rtol=RTOL, atol=ATOL)
    assert_allclose(a2, kinematics_solver.a2, rtol=RTOL, atol=ATOL)
    assert_allclose(a3, kinematics_solver.a3, rtol=RTOL, atol=ATOL)


def test_kinematics_improper_m1_initialization():
    m1 = np.array([0, 0, 0.05, 1])
    a1 = np.array([0.05, 0, -0.035])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(AssertionError):
        kinematics_solver = KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_improper_a1_initialization():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, -0.035, 1])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(AssertionError):
        kinematics_solver = KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_improper_a2_initialization():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, -0.035])
    a2 = np.array([0.09, 0, 0, 1])
    a3 = np.array([0.1, 0, 0])

    with raises(AssertionError):
        kinematics_solver = KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_improper_a3_initialization():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, -0.035])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0, 1])

    with raises(AssertionError):
        kinematics_solver = KinematicsSolver(m1, a1, a2, a3)

def test_kinematics_improper_m1_translation():
    m1 = np.array([1, 0, 0])
    a1 = np.array([1 ,0 ,0])
    a2 = np.array([1 ,0 ,0])
    a3 = np.array([1 ,0 ,0])
    with raises(AssertionError):
        kinematics_solver = KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_improper_a1_translation():
    m1 = np.array([0, 0, 1])
    a1 = np.array([0, 0 ,1])
    a2 = np.array([1, 0 ,0])
    a3 = np.array([1, 0 ,0])
    with raises(AssertionError):
        kinematics_solver = KinematicsSolver(m1, a1, a2, a3)

def test_kinematics_improper_a2_translation():
    m1 = np.array([0, 0, 1])
    a1 = np.array([1 ,0 ,0])
    a2 = np.array([0 ,0 ,1])
    a3 = np.array([1 ,0 ,0])
    with raises(AssertionError):
        kinematics_solver = KinematicsSolver(m1, a1, a2, a3)

def test_kinematics_improper_a3_translation():
    m1 = np.array([0, 0, 1])
    a1 = np.array([1 ,0 ,0])
    a2 = np.array([1 ,0 ,0])
    a3 = np.array([0 ,0 ,1])
    with raises(AssertionError):
        kinematics_solver = KinematicsSolver(m1, a1, a2, a3)

