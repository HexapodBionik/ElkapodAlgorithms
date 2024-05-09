from ...kinematics.kinematics_solvers import KinematicsSolver
from .utils import SplinePolynomial
import numpy as np
import math
from typing import Tuple


class SimpleJointSpaceTrajectory:
    """
    Class used for generation of simple trajectory of hexapod leg in joint space.
    Trajectory generation using 3rd order polynomial.
    """

    def __init__(self, kinematics_solver: KinematicsSolver):
        self._kinematics_solver = kinematics_solver

    def generate_trajectory(self, start_coordinates: np.ndarray,
                            end_coordinates: np.ndarray,
                            velocity_start: np.ndarray,
                            velocity_end: np.ndarray, time_end: float) -> list:
        """
        Generate coefficients for cubic polynomial realizing leg's trajectory
        @param start_coordinates: start coordinates of leg's trajectory
        provided as tuple (x, y, z)
        @param end_coordinates: end coordinates of leg's trajectory
        provided as tuple (x, y, z)
        @param velocity_start: start velocity of each joint in (rad/s)
        @param velocity_end: end velocity of each joint in (rad/s)
        @param time_end: time of movement from start to end
        coordinates in seconds
        @return: list of 3 tuples each containing coefficients
        for cubic polynomial realizing trajectory for each joint
        """
        start_position = np.array([*start_coordinates, 1])
        end_position = np.array([*end_coordinates, 1])

        start_angles_array = self._kinematics_solver.inverse(start_position)
        end_angles_array = self._kinematics_solver.inverse(end_position)

        poly_coefficients = []
        for i in range(3):
            poly_coefficients.append(
                self._generate_polynomial_coefficients(start_angles_array[i],
                                                       end_angles_array[i],
                                                       float(velocity_start[i]),
                                                       float(velocity_end[i]),
                                                       time_end))

        return poly_coefficients

    def _generate_polynomial_coefficients(self, start_angle: float,
                                          end_angle: float,
                                          start_velocity: float,
                                          end_velocity: float, end_time: float) -> np.ndarray:
        """
        Solve system of equations and get the coefficients
        for a cubic polynomial
        """
        A = np.array([[1, 0, 0, 0],
                      [1, end_time, pow(end_time, 2), pow(end_time, 3)],
                      [0, 1, 0, 0],
                      [0, 1, 2 * end_time, 3 * pow(end_time, 2)]]
                     )
        b = np.array([start_angle, end_angle, start_velocity, end_velocity])

        a = np.linalg.solve(A, b)
        return a

class ViaPointsJointSpaceTrajectory:
    def __init__(self, kinematics_solver: KinematicsSolver):
        self._kinematics_solver = kinematics_solver
        self._poly_deg = 3

    def generate_trajectory(self, coordinates: np.ndarray, velocities: np.ndarray, time_steps: np.ndarray) -> list:
        thetas1 = []
        thetas2 = []
        thetas3 = []
        for coordinate in coordinates:
            start_position = np.array([*coordinate, 1])
            angles = self._kinematics_solver.inverse(start_position)
            thetas1.append(angles[0])
            thetas2.append(angles[1])
            thetas3.append(angles[2])

        joint_angles = [thetas1, thetas2, thetas3]

        polys = []
        for i in range(3):
            coefficients = self._generate_polynomial_coefficients(joint_angles[i], velocities[i], time_steps)
            coeffs_split = np.array_split(coefficients, len(coefficients) // (self._poly_deg+1))
            coeffs_split = [coeffs.reshape((4,)) for coeffs in coeffs_split]
            polys.append(SplinePolynomial(coeffs_split, time_steps))

        return polys

    def _generate_polynomial_coefficients(self, angles: np.ndarray, velocities: np.ndarray,  time_steps: np.ndarray) -> np.ndarray:
        polys = len(time_steps)
        # A = np.zeros((polys*(self._poly_deg+1)))
        A = np.zeros((polys*(self._poly_deg+1), polys*(self._poly_deg+1)))

        # Fill the A matrix with coefficients from position equations
        for i in range(polys):
            A[i*2, i*4] = 1
            for j in range(self._poly_deg+1):
                A[i * 2 + 1, i * 4 + j] = pow(time_steps[i], j)

        A[2*polys, 1] = 1
        for j in range(self._poly_deg):
            A[2*polys + 1, (polys - 1) * 4 + 1 + j] = (1+j)*pow(time_steps[-1], j)

        for i in range(polys-1):
            for j in range(self._poly_deg):
                A[polys*2+2+i, i*4+1 + j] = (1+j)*pow(time_steps[i], j)
            A[polys*2+2+i, 1 + (1+i)*4] = -1

        for i in range(polys-1):
            for j in range(self._poly_deg-1):
                A[polys*3+1+i, i*4+2 + j] = math.factorial(2+j) * pow(time_steps[i], j)
            A[polys*3+1+i, 2 + (1+i)*4] = -2

        b = np.zeros((polys*4, 1))
        b[0] = angles[0]
        for i in range(len(angles)-2):
            b[1+i*2] = angles[1+i]
            b[2+i*2] = angles[1+i]

        b[1+(len(angles)-2)*2] = angles[-1]
        b[2 + (len(angles) - 2) * 2] = velocities[0]
        b[3 + (len(angles) - 2) * 2] = velocities[1]

        a = np.linalg.solve(A, b)
        return a


class LSPBTwoPoints:
    def __init__(self, kinematics_solver: KinematicsSolver):
        self._kinematics_solver = kinematics_solver
        self._section_lengths = [3, 2, 3]

    def generate_trajectory(self, start_coordinates: np.ndarray,
                            end_coordinates: np.ndarray,
                            velocity_start: np.ndarray,
                            velocity_end: np.ndarray, max_accel: np.ndarray,  time_end: float) -> list:

        start_position = np.array([*start_coordinates, 1])
        end_position = np.array([*end_coordinates, 1])

        start_angles_array = self._kinematics_solver.inverse(start_position)
        end_angles_array = self._kinematics_solver.inverse(end_position)

        polys = []
        for i in range(3):
            coefficients, time_steps = self._generate_polynomial_coefficients(start_angles_array[i], end_angles_array[i], velocity_start[i], velocity_end[i], max_accel[i], time_end)
            coeffs_split = np.split(coefficients, np.cumsum(self._section_lengths)[:-1])

            coeffs_split = [coeffs.reshape((self._section_lengths[i],)) for i, coeffs in enumerate(coeffs_split)]
            polys.append(SplinePolynomial(coeffs_split, time_steps, [0, 0, 0]))

        return polys

    def minimum_acceleration_for_blends(self, start_position: float, end_position: float, movement_time: float) -> float:
        return 4*(end_position - start_position)/pow(movement_time, 2)

    def _generate_polynomial_coefficients(self, start_angle: float,
                                          end_angle: float,
                                          start_velocity: float,
                                          end_velocity: float, max_accel: float, end_time: float) -> Tuple[np.ndarray, list]:

        t_b = end_time/2 - np.sqrt(pow(max_accel, 2)*pow(end_time, 2) - 4*float(max_accel)*(end_angle-start_angle))/(2*max_accel)

        if t_b > 0:
            time_steps = [t_b, end_time - 2 * t_b, end_time - t_b]

            A = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1, end_time, pow(end_time, 2)],
                          [1, t_b, pow(t_b, 2), -1, -t_b, 0, 0, 0],
                          [0, 0, 0, 1, end_time - t_b, -1, t_b - end_time,
                           -pow(end_time - t_b, 2)],
                          [0, 1, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 1, 2 * end_time],
                          [0, 1, 2 * t_b, 0, -1, 0, 0, 0],
                          [0, 0, 0, 0, 1, 0, -1, -2 * (end_time - t_b)]
                          ]
                         )
            b = np.array(
                [start_angle, end_angle, 0, 0, start_velocity, end_velocity, 0,
                 0])

            a = np.linalg.solve(A, b)
        else:
            a_21 = (end_angle - start_angle)/end_time
            a_20 = end_angle - a_21*end_time
            a = np.array([0, 0, 0, a_20, a_21, 0, 0, 0])
            time_steps = [0, end_time, 0]

        return a, time_steps