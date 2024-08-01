import numpy as np
from numpy.testing import assert_allclose
from ..stability_check import StabilityCheck


def test_zero_in():
    coordinates = np.array([[1,0,-1], [0,1,-1], [-1,-1,-1], [2,3,0], [0,0,-0.5], [3,3,3]])
    s = StabilityCheck()
    assert(s.check_stability(coordinates) == True)


def test_zero_edge():
    coordinates = np.array([[0,2,-1], [0,-2,-1], [3,-2,-1], [3,-2,-1], [0,0,-0.5], [3,3,3]])
    s = StabilityCheck()
    assert(s.check_stability(coordinates) ==False)
    

def test_zero_outside():
    coordinates = np.array([[-1,-2,-1], [0,3,-1], [-1,-1,-1], [2,3,0], [0,0,-0.5], [3,3,3]])
    s = StabilityCheck()
    assert(s.check_stability(coordinates) == False)


def test_zero_in_2():
    coordinates = np.array([[-0.7,-1, 0.5], [-0.5,1.5,0.5], [0.5,-0.5,0.5], [0.7,2,0.5], [0,0,1], [3,3,3]])
    s = StabilityCheck()
    assert(s.check_stability(coordinates) == True)


def test_one_in():
    coordinates = np.array([[-4.8, 3.2, -1], [1, 3.9, -1], [0, 1, -1], [1.3, -4.8, -1], [-2.3, -5.6, -1], [-5.5, -2.4, -1]])
    s = StabilityCheck(1)
    assert(s.check_stability(coordinates) == True)


def test_one_edge():
    coordinates = np.array([[-1,-1,0], [3,-1,0], [-1,3,0], [2,3,1], [4,4,0.6], [2,1,0.1]])
    s = StabilityCheck(1)
    assert(s.check_stability(coordinates) == False)


def test_one_outside():
    coordinates = np.array([[-2,-4,0], [2,-3.5,0], [-2,-0.1,0], [2,0,0], [4,4,0.6], [2,1,0.1]])
    s = StabilityCheck(1)
    assert(s.check_stability(coordinates) == False)
    

def test_one_outside_margin():
    coordinates = np.array([[-2,-4,0], [2,-3.5,0], [-2,-0.5,0], [2,1,0], [4,4,0.6], [2,1,0.1]])
    s = StabilityCheck(1)
    assert(s.check_stability(coordinates) == False)
    
    
def test_with_distance_outside():
    coordinates = np.array([[-1,-2,-1], [0,3,-1], [-1,-1,-1], [2,3,0], [0,0,-0.5], [3,3,3]])
    s = StabilityCheck()
    assert(s.check_stability_with_minimal_distance(coordinates)[1] == None)
    
    
def test_with_distance_margin():
    coordinates = np.array([[0,2,-1], [0,-2,-1], [3,-2,-1], [3,-2,-1], [0,0,-0.5], [3,3,3]])
    s = StabilityCheck()
    assert(s.check_stability_with_minimal_distance(coordinates)[1] == 0.0)
    
    
def test_with_distance_in():
    coordinates = np.array([[1,0,-1], [0,1,-1], [-1,-1,-1], [2,3,0], [0,0,-0.5], [3,3,3]])
    s = StabilityCheck()
    assert(s.check_stability_with_minimal_distance(coordinates)[0] == True)
    assert(s.check_stability_with_minimal_distance(coordinates)[1] == 0.4472135954999579)
    

def test_with_distance_outside_margin():
    coordinates = np.array([[1,0,-1], [0,1,-1], [-1,-1,-1], [2,3,0], [0,0,-0.5], [3,3,3]])
    s = StabilityCheck(0.5)
    assert(s.check_stability_with_minimal_distance(coordinates)[0] == False)
    assert_allclose(s.check_stability_with_minimal_distance(coordinates)[1], 0.44721359)
    
    
def test_with_distance_in_margin():
    coordinates = np.array([[-4.8, 3.2, -1], [1.3, 3.9, -1], [0, 1, -1], [1.3, -4.8, -1], [-2.3, -5.6, -1], [-5.5, -2.4, -1]])
    s = StabilityCheck(1)
    assert(s.check_stability_with_minimal_distance(coordinates)[0] == True)
    assert_allclose(s.check_stability_with_minimal_distance(coordinates)[1], 1.3)