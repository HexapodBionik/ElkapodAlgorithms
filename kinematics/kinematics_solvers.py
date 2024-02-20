import numpy as np
from kinematics.kinematics_utils import homogeneous_transform_matrix, rot_z, rot_x
from kinematics.kinematics_exceptions import (InvalidInitVectorElements, InvalidInitVectorShape, InvalidInitVectorLength,
                                              InvalidInitVector)


class KinematicsSolver:
    """
    Class storing parameters of the
    robotic leg of Hexapod Robot.
    The leg has 3 degrees of freedom.
    """

    def __init__(self, m1: np.ndarray, a1: np.ndarray, a2: np.ndarray, a3: np.ndarray):
        """
        @param m1: Translation from world origin to J1
        @param a1: Link 1 (between J1 and J2)
        @param a2: Link 2 (between J2 and J3)
        @param a3: Link 3 (between J3 and FCP)
        """

        self.m1 = m1
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3

    @staticmethod
    def _check_vector_dimensions(vector: np.ndarray) -> None:
        if not isinstance(vector, np.ndarray):
            raise InvalidInitVector("Improper init vector type! Should be equal to np.ndarray!")
        if vector.ndim != 1:
            raise InvalidInitVectorShape("Improper vector shape! Ndim should be equal to 1!")
        if len(vector) != 3:
            raise InvalidInitVectorLength("Improper vector length! Vector length should equal to 3!")

    @staticmethod
    def _check_vector_numbers(vector: np.ndarray, scalar_index: int) -> None:
        vector_indexes = [x for x in range(len(vector)) if x != scalar_index]
        for index in vector_indexes:
            if vector[index] != 0:
                raise InvalidInitVectorElements(f"Invalid scalar's value! Scalar at index {index} should be equal to 0!")

        
    @property
    def a1(self) -> np.ndarray:
        return self._a1
    
    @a1.setter
    def a1(self, new_a1: np.ndarray) -> None:
        self._check_vector_dimensions(new_a1)
        self._check_vector_numbers(new_a1, scalar_index=0)

        self._a1 = new_a1

    @property
    def a2(self) -> np.ndarray:
        return self._a2

    @a2.setter
    def a2(self, new_a2: np.ndarray) -> None:
        self._check_vector_dimensions(new_a2)
        self._check_vector_numbers(new_a2, scalar_index=0)

        self._a2 = new_a2

    @property
    def a3(self) -> np.ndarray:
        return self._a3

    @a3.setter
    def a3(self, new_a3: np.ndarray) -> None:
        self._check_vector_dimensions(new_a3)
        self._check_vector_numbers(new_a3, scalar_index=0)

        self._a3 = new_a3

    @property
    def m1(self) -> np.ndarray:
        return self._m1

    @m1.setter
    def m1(self, new_m1: np.ndarray) -> None:
        self._check_vector_dimensions(new_m1)
        self._check_vector_numbers(new_m1, scalar_index=2)

        self._m1 = new_m1

    def forward(self, q: np.ndarray) -> np.ndarray:
        """
        Forward Kinematics

        @param q: Vector of joint angles [q1,q2,q3] in degrees
        @return: Position of leg's foot center point as [x,y,z]
        """
        q_rad = [np.deg2rad(x) for x in q]
        
        rot_0_1 = rot_z(q_rad[0])
        rot_1_2 = rot_x(np.pi/2) @ rot_z(q_rad[1])
        rot_2_3 = rot_z(q_rad[2])
        rot_3_4 = rot_z(0)

        trans_0_1 = np.array([0, 0, self._m1[2]])
        trans_1_2 = self._a1
        trans_2_3 = self._a2
        trans_3_4 = self._a3

        t_0_1 = homogeneous_transform_matrix(rot_0_1, trans_0_1)
        t_1_2 = homogeneous_transform_matrix(rot_1_2, trans_1_2)
        t_2_3 = homogeneous_transform_matrix(rot_2_3, trans_2_3)
        t_3_4 = homogeneous_transform_matrix(rot_3_4, trans_3_4)

        t_0_4 = t_0_1 @ t_1_2 @ t_2_3 @ t_3_4

        vector = np.array([0, 0, 0, 1])

        fcp = t_0_4 @ vector
        return fcp[:3]

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
        l = np.sqrt(x ** 2 + y ** 2) - self._a1[0]
        z = z - self._m1[2] - self._a1[2]
        P = np.sqrt(l ** 2 + z ** 2)
        q2 = np.arctan2(z, l) + np.arccos((P ** 2 + self._a2[0] ** 2 - self._a3[0] ** 2) / (2 * P * self._a2[0]))
        q3 = np.arccos((self._a2[0] ** 2 + self._a3[0] ** 2 - P ** 2) / (2 * self._a2[0] * self._a3[0])) - np.pi

        return np.array([np.rad2deg(x) for x in [q1, q2, q3]])
