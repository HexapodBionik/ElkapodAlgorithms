import numpy as np


def rot_x(alpha: float) -> np.ndarray:
    """
    Create rotation matrix by
    angle alpha around axis X.

    @param alpha: Some angle
    @return: Rotation matrix
    """
    return np.array([[1, 0, 0],
                     [0, np.cos(alpha), -np.sin(alpha)],
                     [0, np.sin(alpha), np.cos(alpha)]
                     ])


def rot_y(alpha: float) -> np.ndarray:
    """
    Create rotation matrix by
    angle alpha around axis Y.

    @param alpha: Some angle
    @return: Rotation matrix
    """
    return np.array([[np.cos(alpha), 0, np.sin(alpha)],
                     [0, 1, 0],
                     [-np.sin(alpha), 0, np.cos(alpha)]
                     ])


def rot_z(alpha: float) -> np.ndarray:
    """
    Create rotation matrix by
    angle alpha around axis Z.

    @param alpha: Some angle
    @return: Rotation matrix
    """
    return np.array([[np.cos(alpha), -np.sin(alpha), 0],
                     [np.sin(alpha), np.cos(alpha), 0],
                     [0, 0, 1]
                     ])


def homogeneous_transform_matrix(rotation: np.ndarray,
                                 translation: np.ndarray) -> np.ndarray:
    """
    Returns homogeneous transform matrix for 3D operations,
    robotics version (perspective set to vector of zeros, scale set to 1)

    @param rotation: 3x3 matrix, positively determined
    @param translation: 1x3 matrix, [dx, dy, dz]
    """

    htm = np.array([[*rotation[0], translation[0]],
                    [*rotation[1], translation[1]],
                    [*rotation[2], translation[2]],
                    [0, 0, 0, 1]])
    return htm


"""
Other functions left for backward compatibility
"""


def se3_norm(v):
    """
    Length of vector from special
    euclidean group SE3.

    @param v: Some vector from SE3
    @return: Length of v
    """
    assert v.ndim == 1
    assert len(v) == 4
    assert v[3] == 1

    return np.linalg.norm(
        np.array([v[0], v[1], v[2]]))


def trans(v):
    """
    Create translation matrix by
    vector v.

    @param v: Some vector from SE3
    @return: Translation by v matrix
    """
    return np.array([[1, 0, 0, v[0]],
                     [0, 1, 0, v[1]],
                     [0, 0, 1, v[2]],
                     [0, 0, 0, v[3]]])


def adjust_float_point_error(arg: float):
    sign = np.sign(arg)
    if np.isclose(1, abs(arg), 1e-9):
        return sign*(abs(arg)-0.00000000000001)
    return arg
