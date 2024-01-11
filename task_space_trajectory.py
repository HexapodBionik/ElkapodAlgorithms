import numpy as np
from kinematics.kinematics import KinematicsSolver


class TaskSpaceTrajectory:
    period = 0.05

    def __init__(self, kinematics_solver: KinematicsSolver):
        self._kinematics_solver = kinematics_solver

    def generate_trajectory(self, start_position: np.ndarray, end_position: np.ndarray, start_velocity: np.ndarray,
                            end_velocity: np.ndarray, move_time: int):
        # Todo verify passed arguments
        return self._get_coefficients_of_trajectory(start_position, end_position, start_velocity, end_velocity,
                                                    move_time)

    def get_joint_trajectory(self, move_time: int, trajectory_coefficients: np.ndarray):
        # Todo verify trajectory_coefficients
        time_stamps = np.arange(0, move_time, TaskSpaceTrajectory.period)
        Q = []
        for t in time_stamps:
            x = np.polyval(trajectory_coefficients[:, 0], t)
            y = np.polyval(trajectory_coefficients[:, 1], t)
            z = np.polyval(trajectory_coefficients[:, 2], t)
            Q.append(self._kinematics_solver.inverse(np.array([x, y, z, 1])))
        return Q

    @staticmethod
    def _get_coefficients_of_trajectory(start_position, end_position, start_velocity, end_velocity, move_time):
        a0 = start_position
        a1 = start_velocity
        a3 = (1 / (move_time ** 3)) * (move_time * (end_velocity - start_velocity) - 2 * (
                end_position - start_position - move_time * start_velocity))
        a2 = (1 / (move_time ** 2)) * (end_position - a0 - a1 * move_time - a3 * (move_time ** 3))
        return np.array([a3, a2, a1, a0])

    @staticmethod
    def _convert_position(position):
        assert len(position) == 3
        return np.array([position[0], position[1], position[2], 1])
