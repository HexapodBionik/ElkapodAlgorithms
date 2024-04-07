import numpy as np
# from numpy.testing import assert_allclose
from ..stability_check import StabilityCheck


def test_zero_in():
    coordinates = np.array([[1,0,-1], [0,1,-1], [-1,-1,-1], [2,3,0], [0,0,-0.5], [3,3,3]])
    s = StabilityCheck(0)
    assert(s.check_stability(coordinates) == True)


def test_zero_edge():
    coordinates = np.array([[0,2,-1], [0,-2,-1], [3,-2,-1], [3,-2,-1], [0,0,-0.5], [3,3,3]])
    s = StabilityCheck(0)
    assert(s.check_stability(coordinates) ==False)
    

def test_zero_outside():
    coordinates = np.array([[-1,-2,-1], [0,3,-1], [-1,-1,-1], [2,3,0], [0,0,-0.5], [3,3,3]])
    s = StabilityCheck(0)
    assert(s.check_stability(coordinates) == False)


def test_in():
    coordinates = np.array([[-0.7,-1, 0.5], [-0.5,1.5,0.5], [0.5,-0.5,0.5], [0.7,2,0.5], [0,0,1], [3,3,3]])
    s = StabilityCheck(0)
    assert(s.check_stability(coordinates) == True)


def test_edge():
    coordinates = np.array([[-1,-1,0], [3,-1,0], [-1,3,0], [2,3,1], [4,4,0.6], [2,1,0.1]])
    s = StabilityCheck(1)
    assert(s.check_stability(coordinates) == False)


def test_outside():
    coordinates = np.array([[-2,-4,0], [2,-3.5,0], [-2,-0.1,0], [2,0,0], [4,4,0.6], [2,1,0.1]])
    s = StabilityCheck(1)
    assert(s.check_stability(coordinates) == False)
    