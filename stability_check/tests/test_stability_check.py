import numpy as np
# from numpy.testing import assert_allclose
from ..stability_check import StabilityCheck

def test_1():
    coordinates = np.array([[1,0,-1], [0,1,-1], [-1,-1,-1], [2,3,0], [0,0,-0.5], [3,3,3]])
    s = StabilityCheck(0)
    assert(s.check_stability(coordinates) == True)
