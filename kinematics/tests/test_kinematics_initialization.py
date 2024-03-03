from pytest import raises
from MotionPlanning.kinematics.kinematics_solvers import KinematicsSolver
from MotionPlanning.kinematics.kinematics_exceptions import InvalidInitVector
import numpy as np
from numpy.testing import assert_allclose
from MotionPlanning.kinematics.tests import ATOL, RTOL


def test_kinematics_proper_initialization():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    kinematics_solver = KinematicsSolver(m1, a1, a2, a3)

    assert_allclose(m1, kinematics_solver.m1, rtol=RTOL, atol=ATOL)
    assert_allclose(a1, kinematics_solver.a1, rtol=RTOL, atol=ATOL)
    assert_allclose(a2, kinematics_solver.a2, rtol=RTOL, atol=ATOL)
    assert_allclose(a3, kinematics_solver.a3, rtol=RTOL, atol=ATOL)


def test_kinematics_invalid_m1_type():
    m1 = 0
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_invalid_a1_type():
    m1 = np.array([0, 0, 0.05])
    a1 = 0
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_invalid_a2_type():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = 0
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_invalid_a3_type():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = 0

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_m1_too_few_number_of_dimensions():
    m1 = np.array(0)
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_m1_too_many_number_of_dimensions():
    m1 = np.array([0, 0, 0.05, 1])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_m1_improper_elements_values_1():
    m1 = np.array([0.5, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_m1_improper_elements_values_2():
    m1 = np.array([0, 0.5, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a1_too_few_number_of_dimensions():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array(1)
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a1_too_many_number_of_dimensions():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0, 1])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a1_improper_elements_values_1():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0.5])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a1_improper_elements_values_2():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0.5, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a2_too_few_number_of_dimensions():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array(1)
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a2_too_many_number_of_dimensions():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0, 1])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a2_improper_elements_values_1():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0.1, 0])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a2_improper_elements_values_2():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0.1])
    a3 = np.array([0.1, 0, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a3_too_few_number_of_dimensions():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array(1)

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a3_too_many_number_of_dimensions():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0, 1])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a3_improper_elements_values_1():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0.1, 0])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_a3_improper_elements_values_2():
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.1, 0, 0.1])

    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_improper_m1_translation():
    m1 = np.array([1, 0, 0])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])
    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_improper_a1_translation():
    m1 = np.array([0, 0, 1])
    a1 = np.array([0, 0, 1])
    a2 = np.array([1, 0, 0])
    a3 = np.array([1, 0, 0])
    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_improper_a2_translation():
    m1 = np.array([0, 0, 1])
    a1 = np.array([1, 0, 0])
    a2 = np.array([0, 0, 1])
    a3 = np.array([1, 0, 0])
    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)


def test_kinematics_improper_a3_translation():
    m1 = np.array([0, 0, 1])
    a1 = np.array([1, 0, 0])
    a2 = np.array([1, 0, 0])
    a3 = np.array([0, 0, 1])
    with raises(InvalidInitVector):
        KinematicsSolver(m1, a1, a2, a3)
