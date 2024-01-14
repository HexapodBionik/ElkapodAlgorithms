import numpy as np
from typing import Tuple


def acquire_trajectory_functions(poly_coefficients: np.ndarray) \
        -> Tuple[np.poly1d, np.poly1d, np.poly1d]:
    position_polynomial = np.poly1d(np.flip(poly_coefficients))
    velocity_polynomial = np.polyder(position_polynomial)
    acceleration_polynomial = np.polyder(velocity_polynomial)
    return position_polynomial, velocity_polynomial, acceleration_polynomial


class SplinePolynomial:
    def __init__(self, coefficients_list: np.ndarray, poly_periods: np.ndarray,
                 poly_start_times=None):
        self._coefficients_list = coefficients_list
        self._poly_periods = poly_periods
        self._time_limits = [sum(self._poly_periods[:i]) for i in
                             range(1,
                                   len(self._poly_periods) + 1)]

        if poly_start_times is None:
            self._poly_start_times = self._time_limits
        else:
            self._poly_start_times = poly_start_times

    def __call__(self, time_steps: np.ndarray) -> float:
        values = []
        for time_step in time_steps:
            poly, polynumber = self._select_polynomial(time_step)

            values.append(
                self._generate_polynomial_value(poly, time_step, polynumber))

        return np.array(values)

    def _select_polynomial(self, time_step: float) -> Tuple[
                                                          np.poly1d, int] | None:
        for i, time_limit in enumerate(self._time_limits):
            if time_step < time_limit:
                polynumber = i

                poly = np.poly1d(np.flip(self._coefficients_list[polynumber]))
                return poly, polynumber
        return None

    def _generate_polynomial_value(self, poly: np.poly1d, time_step: float,
                                   polynumber: int) -> float:
        if polynumber == 0:
            return poly(time_step)
        else:
            return poly(
                time_step - self._poly_start_times[polynumber - 1])

    def get_velocity(self, time_steps: np.ndarray) -> float:
        values = []
        for time_step in time_steps:
            poly, polynumber = self._select_polynomial(time_step)
            poly = np.polyder(poly)

            values.append(
                self._generate_polynomial_value(poly, time_step, polynumber))
        return np.array(values)

    def get_acceleration(self, time_steps: np.ndarray) -> float:
        values = []
        for time_step in time_steps:
            poly, polynumber = self._select_polynomial(time_step)
            poly = np.polyder(poly, 2)

            values.append(
                self._generate_polynomial_value(poly, time_step, polynumber))
        return np.array(values)
