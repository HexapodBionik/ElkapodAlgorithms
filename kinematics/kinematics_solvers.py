import numpy as np
import math
from typing import Tuple
from kinematics.kinematics_utils import homogeneous_transform_matrix, rot_z, rot_x


class KinematicsSolver:
    """
    Class storing parameters of the
    robotic leg of Hexapod Robot.
    The leg has 3 degrees of freedom.
    """

    def __init__(self, mount_t: np.ndarray, t1: np.ndarray, t2: np.ndarray, t3: np.ndarray):
        """
        @param mount_t: Translation from world origin to J1
        @param t1: Translation 1 (between J1 and J2)
        @param t2: Translation 2 (between J2 and J3)
        @param t3: Translation 3 (between J3 and FCP)
        """

        assert mount_t.ndim == 1
        assert t1.ndim == 1
        assert t2.ndim == 1
        assert t3.ndim == 1
        assert len(mount_t) == 3
        assert len(t1) == 3
        assert len(t2) == 3
        assert len(t3) == 3

        self._mount_t = mount_t
        self._t1 = t1
        self._t2 = t2
        self._t3 = t3

    def forward(self, q: np.ndarray) -> np.ndarray:
        """
        Forward Kinematics

        @param q: Vector of joint variables [q1,q2,q3]
        @return: Position of leg's foot center point as [x,y,z]
        """
        rot_0_1 = rot_z(q[0])
        rot_1_2 = rot_x(np.pi/2)@rot_z(q[1])
        rot_2_3 = rot_z(q[2])
        rot_3_4 = rot_z(0)

        trans_0_1 = np.array([0, 0, self._mount_t[2]])
        trans_1_2 = self._t1
        trans_2_3 = self._t2
        trans_3_4 = self._t3

        t_0_1 = homogeneous_transform_matrix(rot_0_1, trans_0_1)
        t_1_2 = homogeneous_transform_matrix(rot_1_2, trans_1_2)
        t_2_3 = homogeneous_transform_matrix(rot_2_3, trans_2_3)
        t_3_4 = homogeneous_transform_matrix(rot_3_4, trans_3_4)

        t_0_4 = t_0_1 @ t_1_2 @ t_2_3 @ t_3_4

        vector = np.array([0, 0, 0, 1])

        fcp = t_0_4 @ vector
        return fcp[:2]

    def inverse(self, p: np.ndarray) -> np.ndarray:
        """
        Inverse Kinematics

        @param p: Desired position of arm's end | (x, y, z)
        @return: Angles to be set on servos
        """

        x = p[0]
        y = p[1]
        z = p[2]

        q1 = np.arctan2(y, x)
        l = np.sqrt(x ** 2 + y ** 2) - self._t1[0]
        z = z - self._mount_t[2] - self._t1[2]

        q2 = np.arctan2(z, l) + np.arccos((l ** 2 + self._t2[0] ** 2 - self._t3[0] ** 2) / (2 * l * self._t2[0]))
        q3 = np.arccos((self._t2[0] ** 2 + self._t3[0] ** 2 - l ** 2) / (2 * self._t2[0] * self._t3[0])) - np.pi

        return np.array([q1, q2, q3])
