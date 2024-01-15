import numpy as np
from kinematics.kinematics import KinematicsSolver


class TaskSpaceTrajectory:
    """
    Class used to generate a trajectory of hexapod's limb in task space
    Trajectory generation using 3rd order polynomial
    """
    period = 0.05

    def __init__(self, kinematics_solver: KinematicsSolver):
        self._kinematics_solver = kinematics_solver

    def get_trajectory(self, start_position: np.ndarray, end_position: np.ndarray, start_velocity: np.ndarray,
                       end_velocity: np.ndarray, move_time: float|int) -> list[tuple]:
        """Generates a list of subsequent joint position vectors realizing  trajectory in task space
        @param start_position: start position of the limb, provided as a numpy array [x,y,z]
        @param end_position: end position of the limb, provided as a numpy array [x,y,z]
        @param start_velocity: starting velocity of the limb, provided as numpy array [vx,vy,vz]
        using zeros is advised
        @param end_velocity: ending velocity of the limb, provided as numpy array [vx,vy,vz] using zeros is advised
        @param move_time: time in seconds in which tcp should complete the movement, provided as a float or int
        @return: list of 3 element tuples, subsequent joint vectors in every period."""

        coefficients = self._generate_trajectory_in_task_space(start_position, end_position, start_velocity,
                                                               end_velocity, move_time)

        joint_vectors = self._get_joint_trajectory(move_time, coefficients)

        return joint_vectors

    def _generate_trajectory_in_task_space(self, start_position: np.ndarray, end_position: np.ndarray,
                                           start_velocity: np.ndarray,
                                           end_velocity: np.ndarray, move_time: int):
        """
        Generates polynomials in form of coefficients for each coordinate realizing the task space trajectory in given time.
        @param start_position: start position of the limb, provided as a numpy array [x,y,z]
        @param end_position: end position of the limb, provided as a numpy array [x,y,z]
        @param start_velocity: starting velocity of the limb, provided as numpy array [vx,vy,vz]
        using zeros is advised
        @param end_velocity: ending velocity of the limb, provided as numpy array [vx,vy,vz] using zeros is advised
        @param move_time: time in seconds in which tcp should complete the movement, provided as a float or int"""
        # Todo verify passed arguments
        return self._get_coefficients_of_trajectory(start_position, end_position, start_velocity, end_velocity,
                                                    move_time)

    def _get_joint_trajectory(self, move_time: int|float, trajectory_coefficients: np.ndarray) -> list:
        """Translates continuous realization of task space trajectory into discrete joint positions
        @param move_time: time in seconds in which movement should be completed
        @param trajectory_coefficients: coefficients of polynomials for each coordinate realizing given task space trajectory
        @return: list of 3 element tuples, subsequent joint vectors in every period. """
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
