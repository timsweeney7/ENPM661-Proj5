import numpy as np
import cv2 as cv
import random


# map dimensions
X_MAX = 300
Y_MAX = 300
SCALE_FACTOR = 2
X_MAX_SCALED = X_MAX * SCALE_FACTOR
Y_MAX_SCALED = Y_MAX * SCALE_FACTOR

BLUE = (255, 0, 0)
DARK_GREEN = (15, 168, 33)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
YELLOW = (9, 227, 212)
BLACK = (0, 0, 0)
GRAY = (199, 198, 195)
WHITE = (255,255,255)

# for coordinates
X = 0
Y = 1

#TODO MOVE THESE INTO A FUNCTION
obstacle_points = set()  # used to quickly look up if a point is in an obstacle
map_points = set()       # used to quickly look up if a point is in the map



def draw_map_1(num_of_rectangles):
    # Background
    background_color = WHITE
    map = np.zeros((Y_MAX_SCALED, X_MAX_SCALED, 3), np.uint8)
    map[:] = background_color

    for i in range(0,num_of_rectangles):
        first_point_x = random.randint(0, X_MAX_SCALED)
        first_point_y = random.randint(0, Y_MAX_SCALED)
        first_point = [first_point_x, first_point_y]
        x_increase = random.randint(4,10)
        y_increase = random.randint(4,10)
        x_increase_scaled = x_increase * SCALE_FACTOR
        y_increase_scaled = y_increase * SCALE_FACTOR
        second_point = [first_point_x + x_increase_scaled, first_point_y + y_increase_scaled]
        if second_point[1] > X_MAX_SCALED:
            second_point[1] = X_MAX_SCALED
        if second_point[0] > Y_MAX_SCALED:
            second_point[0] = Y_MAX_SCALED

        cv.rectangle(map, first_point, second_point, BLACK, -1)

    return map

"""
determine_valid_point

Determines if a given set of coordinates is in free space or in obstacle space

color_map:   numpy_array of a color map. map is 3 dimensions [y, x, [color]]
coordinates: set of xy coordinates [x, y]

"""
def determine_valid_point(color_map, coordinates):
    if not __point_is_inside_map(coordinates[X], coordinates[Y]):
        return False
    if color_map[coordinates[Y], coordinates[X]] == WHITE:
        return True
    elif color_map[coordinates[Y], coordinates[X]] == BLACK:
        return False
    else:
        raise Exception("determine_valid_point was passed an invalid argument")


def __point_is_inside_map(x, y):
    if (x > 600) or (x < 0):
        return False
    elif (y > 250) or (y < 0):
        return False
    else:
        return True

def __add_point(x, y, map, color):
    map[y, x] = color
    return map


def __draw_line(p1, p2, map, color):
    pts = np.array([[p1[0], p1[1]], [p2[0], p2[1]]],
                   np.int32)
    cv.fillPoly(map, [pts], color)


def draw_node(child_coordinates, parent_coordinates, map, color):

    child_coordinates = tuple(int(SCALE_FACTOR * _) for _ in child_coordinates)
    cv.circle(map, child_coordinates, radius=3, color=color, thickness=-1)

    if (parent_coordinates is not None):
        parent_coordinates = tuple(int(SCALE_FACTOR * _ ) for _ in parent_coordinates)
        cv.circle(map, parent_coordinates, radius=3, color=color, thickness=-1)
        __draw_line(child_coordinates, parent_coordinates, map, color)


if __name__ == "__main__":
    color_map = draw_map_1(200)
    cv.imshow('Informed RRT* Algorith', color_map)
    cv.waitKey(0)