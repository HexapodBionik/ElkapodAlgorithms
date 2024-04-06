import numpy as np
from .stability_math import lowest_z_coordinates, convex_polygon_points, calculate_minimal_distance_from_edge


class StabilityCheck:
    def __init__(self, stability_margin: float):
        self.stability_margin = stability_margin
    
    def set_margin(self, new_margin: float) -> None:
        self.stability_margin = new_margin
        
    def get_margin(self) -> float:
        return self.stability_margin
    
    def check_stability(self, legs_coordinates: np.ndarray) -> bool:
        touching_points = lowest_z_coordinates(legs_coordinates)
        touching_points.append(np.array([0,0]))
        polygon_points = convex_polygon_points(touching_points)
        if(len([pnt for pnt in polygon_points if (pnt[0] == 0 and pnt[1] == 0)])):
            return False
        minimal_distance = calculate_minimal_distance_from_edge([0,0], polygon_points)
        if minimal_distance < self.stability_margin:
            False
        return True
