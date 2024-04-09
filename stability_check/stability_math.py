import numpy as np
from math import sqrt


def lowest_z_coordinates(legs_coordinates: np.ndarray) -> list:
    touching_legs = [element[:2] for element in legs_coordinates if element[2] == min(legs_coordinates[:,2])]
    return touching_legs


def is_counter_clockwise(p1, p2, p3) -> bool:
    if (p3[1]-p1[1])*(p2[0]-p1[0]) >= (p2[1]-p1[1])*(p3[0]-p1[0]):
        return True
    return False


def convex_polygon_points(coordinates) -> list:
    """
    Solution to Convex Hull problem using Jarvis' March algorithm
    """
    n = len(coordinates)
    coordinates = np.array(coordinates)
    polygon_vertices = [None] * n
    most_left_point = np.where(coordinates[:,0] == np.min(coordinates[:,0]))
    new_vertex = coordinates[most_left_point[0][0]]
    i = 0
    while True:
        polygon_vertices[i] = new_vertex
        endpoint = coordinates[0]
        for j in range(1,n):
            if (endpoint[0] == new_vertex[0] and endpoint[1] == new_vertex[1]) or not is_counter_clockwise(coordinates[j],polygon_vertices[i],endpoint):
                endpoint = coordinates[j]
        i = i + 1
        new_vertex = endpoint
        if endpoint[0] == polygon_vertices[0][0] and endpoint[1] == polygon_vertices[0][1]:
            break
    for i in range(n):
        if polygon_vertices[-1] is None:
            del polygon_vertices[-1]
    return polygon_vertices


def line_from_two_points(p1, p2) -> list:
    A = p1[1] - p2[1]
    B = p2[0] - p1[0]
    C = p2[1]*p1[0] - p1[1]*p2[0]
    return [A, B, C]


def distance_from_line(point, abc) -> float:
    A = abc[0]
    B = abc[1]
    C = abc[2]
    nominator = abs(A*point[0] + B*point[1] + C)
    denominator = sqrt(A**2 + B**2)
    return nominator/denominator


def calculate_minimal_distance_from_edge(point, vertices) -> float:
    distances = []
    edge = line_from_two_points(vertices[-1], vertices[0])
    distances.append(distance_from_line(point, edge))
    for i in range(len(vertices)-1):
        edge = line_from_two_points(vertices[i],vertices[i+1])
        distances.append(distance_from_line(point, edge))
    return min(distances)

