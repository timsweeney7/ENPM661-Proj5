"""
code based on this paper:
https://arxiv.org/pdf/1404.2334.pdf
"""

# rrt_star.py
import mapping
import math
import random
import numpy as np
import cv2 as cv
from mapping import SCALE_FACTOR
import heapq
from heapq import heapify
import time
from copy import deepcopy



class Node:
    def __init__(self):
        self.costToCome = float('inf')
        self.parentCoordinates = []

    def __str__(self):
        return f"Cost To Come: {self.costToCome}  Parent Coordinates: {self.parentCoordinates}"


def create_pixel_info_map(color_map):
    pixel_info_map = np.ndarray.tolist(color_map)

    y_dim_len = color_map.shape[0]
    x_dim_len = color_map.shape[1]

    for x in range(0, x_dim_len):
        for y in range(0, y_dim_len):
            if mapping.point_is_valid(color_map=color_map, coordinates=(x, y)):
                obs = False
            else:
                obs = True
            pixel_info_map[y][x] = {"c2c": float('inf'), "parentCoor": None, "selfCoordinates":(x, y),"obstacle":obs}
    return pixel_info_map


"""
gets a random point in the elipse from start point to end point
start_point - tuple - (x, y)
goal_point - tuple - (x, y)
best_solution - dictionary - Represents a node. {"c2c", "parentCoordinates", "selfCoordinates", "obstacle"}
                             Will be 'None' if no solution has been found yet
"""
def get_random_point (start_point:tuple, goal_point:tuple, best_solution:dict):
    if best_solution is not None:
        x_point, y_point = -1, -1
        # this makes sure the new point is in the bounds of the map
        cost_min = distance(start_point, goal_point) - GOAL_RADIUS
        print("cost min=", cost_min)
        cost_max = best_solution["c2c"]
        print("cost_max=", cost_max)
        print("cbest=", cost_min/cost_max)
        while (cost_min/cost_max < cbest) :
            while(x_point < 0 or x_point > mapping.X_MAX_SCALED-1 or y_point < 0 or y_point > mapping.X_MAX_SCALED-1):
                cost_min = distance(start_point, goal_point) - GOAL_RADIUS
                # print("cost min=", cost_min)
                cost_max = best_solution["c2c"]
                # print("cost_max=", cost_max)
                semi_major_axis = best_solution["c2c"] / 2
                semi_minor_axis = math.sqrt(pow(cost_max, 2) - pow(cost_min, 2))/2
                ellipse_angle = np.arctan2(goal_point[1] - start_point[1], goal_point[0] - start_point[0])
                center_x = (start_point[0] + goal_point[0])/2
                center_y = (start_point[1] + goal_point[1])/2

                theta_random = 2 * np.pi * np.random.rand()
                radius_random = np.sqrt(np.random.rand())
                # Define the rotation matrix for the ellipse
                cos_angle = np.cos(ellipse_angle)
                sin_angle = np.sin(ellipse_angle)
                rot_matrix = np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])

                # Rotate the random point using the same rotation matrix
                rand_point = np.dot(rot_matrix, 
                                    np.array([semi_major_axis * radius_random * np.cos(theta_random), 
                                    semi_minor_axis * radius_random * np.sin(theta_random)]))
                x_point = rand_point[0] + center_x
                y_point = rand_point[1] + center_y
                x_point = int(x_point)
                y_point = int(y_point)
        
            return (x_point, y_point)
        return(None)

    else:
        # generate a random point in the bounds of the map
        x_coord = random.randint(0, mapping.X_MAX_SCALED-1)
        y_coord = random.randint(0, mapping.Y_MAX_SCALED-1)
        rand_pt = (x_coord, y_coord)
        return rand_pt


def distance (pt1, pt2): 
        distance = math.sqrt(pow(pt2[0] - pt1[0], 2) + pow(pt2[1] - pt1[1], 2))
        return distance


def path_is_good(pt1, pt2):
    line = get_line_coordinates(pt1, pt2)
    for point in line:
        point_int = tuple(map(int, point))
        if not mapping.point_is_valid(color_map=color_map, coordinates=point_int):
            return False
    return True


"""
    Returns a list of coordinates between two points (x1, y1) and (x2, y2) using Bresenham's line algorithm.
"""
def get_line_coordinates(p1, p2):
    
    x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
    coordinates = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = -1 if x1 > x2 else 1
    sy = -1 if y1 > y2 else 1
    err = dx - dy
    while x1 != x2 or y1 != y2:
        coordinates.append((x1, y1))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    coordinates.append((x2, y2))
    return coordinates


"""
gets all of the points (explored or not) in a given radius around a given point
"""
def get_points_in_neighborhood(pt, radius):
    points = set()
    x, y = pt[0], pt[1]  # access the x and y coordinates separately
    for i in range(x - radius, x + radius + 1):
        for j in range(y - radius, y + radius + 1):
            if math.sqrt((i - x) ** 2 + (j - y) ** 2) <= radius:
                points.add((i, j))
    points.remove((x,y))
    return points

"""
gets all of the points in a given radius that have been previously explored
"""
def get_neighbor_nodes(pt, radius, explored_nodes): 
    neighborhood = get_points_in_neighborhood(pt,radius)
    nodes_in_neighborhood = []
    for node in explored_nodes: 
        if node["selfCoordinates"] in neighborhood:
            nodes_in_neighborhood.append(node)
    return nodes_in_neighborhood


"""
Given a new point, and a list of old points, determine lowest cost to come to the new point from the old points
"""
def create_new_node(pt, nodes_in_neightborhood):
    temp_queue = []
    for parent_node in nodes_in_neightborhood:
        dist = distance(pt, parent_node["selfCoordinates"])
        c2c = dist + parent_node["c2c"]
        heapq.heappush(temp_queue, (c2c, parent_node["selfCoordinates"]))
    try:
        while(True):
            heapify(temp_queue)
            c2c, best_neighbor = heapq.heappop(temp_queue)
            if path_is_good(pt1= pt, pt2= best_neighbor):
                new_node = {"c2c": c2c, 
                            "parentCoordinates": best_neighbor, 
                            "selfCoordinates": pt, 
                            "obstacle": False}
                
                path = backtrack(new_node, pixel_info_map)
                debug_count = 0
                for node in path:
                    if node["parentCoordinates"] is not None:
                        debug_count += distance(pt1= node["parentCoordinates"], pt2= node["selfCoordinates"])
                # if debug_count != new_node["c2c"]:
                #     # print("gotcha")
                return new_node
    except IndexError:
        return None


def update_neighborhood(new_node, nodes_in_neightborhood, explored_nodes, pixel_map): 
    for node in nodes_in_neightborhood:
        dist = distance(pt1= new_node["selfCoordinates"], pt2=node ["selfCoordinates"])
        tempC2C = dist + new_node["c2c"]
        if tempC2C < node["c2c"]: 
            if path_is_good(pt1=new_node["selfCoordinates"], pt2=node["selfCoordinates"]):
                updated_neighbor = {"c2c": tempC2C, 
                                    "parentCoordinates": new_node["selfCoordinates"], 
                                    "selfCoordinates": node["selfCoordinates"], 
                                    "obstacle": False}
                for explored_node in explored_nodes:
                    if updated_neighbor["selfCoordinates"] == explored_node["selfCoordinates"]:
                        explored_node["c2c"] = updated_neighbor["c2c"]
                        explored_node["parentCoordinates"] = updated_neighbor["parentCoordinates"]
                        x, y = explored_node["selfCoordinates"]
                        pixel_map[y][x] = explored_node


"""
checks the list of solutions and determines which one has the lowest c2c
"""
def get_current_best_solution(solutions, pixel_map):
    min_cost = float('inf')
    best_node = None
    for point in solutions:
        x, y = point
        node = pixel_map[y][x]
        if node["c2c"] < min_cost:
            best_node = node
    return best_node


def explore(pixel_map:list, explored_nodes:list, start_point:tuple, goal_point:tuple, goal_radius, num_of_iterations:int):
    gen_pts_set = set()
    solutions_set = set()
    gen_pts_set.add(start_point)
    solution_path_list = []
    start_time = time.time()
    lowest_cost = float('inf')

    for i in range(0, num_of_iterations):
        if time.time() - start_time >= time_limit:
            break  # time limit reached, break out of the loop
        best_solution = get_current_best_solution(solutions_set, pixel_map)
        new_pt = get_random_point(start_point, goal_point, best_solution)
        if new_pt is None:
            break
        x, y = new_pt
        if new_pt not in gen_pts_set:
            if pixel_map[y][x]["obstacle"] == False:
                # Find the explored point that is closest to the new point
                nodes_in_neighborhood = get_neighbor_nodes(new_pt, rewiring_radius, explored_nodes=explored_nodes_list)
                new_node = create_new_node(new_pt, nodes_in_neighborhood)
                if new_node is not None:
                    explored_nodes.append(new_node)
                    gen_pts_set.add((x, y))  
                    pixel_map[y][x] = new_node
                    update_neighborhood(new_node, nodes_in_neighborhood, explored_nodes, pixel_map)
                    
                    if distance(pt1= new_pt , pt2= goal_point) < goal_radius:
                        solutions_set.add(new_pt)
                        solution_path_list.append(backtrack(new_node, pixel_map))
                    
                    best_solution = get_current_best_solution(solutions_set, pixel_map)
                    if (best_solution is not None) and (best_solution["c2c"] < lowest_cost) :
                        solution = backtrack(best_solution, pixel_map)
                        lowest_cost = best_solution["c2c"]
                        starting_map = deepcopy(color_map)
                        for i in explored_nodes_list:
                            mapping.draw_node(child_coordinates=i["selfCoordinates"], 
                                            parent_coordinates=i["parentCoordinates"], 
                                            map= starting_map, 
                                            color= mapping.BLUE)
                        cv.circle(starting_map, GOAL_POINT, radius=GOAL_RADIUS, color=mapping.GRAY, thickness=-1)
                        for i in solution:
                            mapping.draw_node(child_coordinates=i["selfCoordinates"], \
                                                parent_coordinates=i["parentCoordinates"], \
                                                map= starting_map, 
                                                color= mapping.RED)
                        end_point = solution[-1]
                        mapping.draw_node(child_coordinates=end_point["selfCoordinates"], \
                                            parent_coordinates= None, \
                                            map= starting_map, 
                                            color= mapping.GREEN)
                        cv.imshow('informed RRT* Algorithm', starting_map)
                        cv.waitKey(0)
    return best_solution
                    

def backtrack (last_node:dict, map_:list):
    solution_path = []
    current_node = last_node
    solution_path.append(current_node)
    
    # (C2G, C2C, TC, point_index, (x,y,theta)parent_coordinates, (x,y,theta)coordinates)
    while current_node["parentCoordinates"] is not None:
        x, y = current_node["parentCoordinates"]
        parent_node = map_[y][x]
        current_node = parent_node
        solution_path.append(parent_node)
 
    solution_path.reverse()
    return solution_path 



if __name__ == "__main__":
    print()

    # --- Simulation Setup -----------------------
    X_MAX = mapping.X_MAX
    Y_MAX = mapping.Y_MAX
    start_time = time.time()
    explored_nodes_list = []
    NUM_OF_ITERATIONS = 50000
    START_POINT = (10, 10)
    GOAL_POINT = (250, 250)
    GOAL_RADIUS = 10
    rewiring_radius = 20
    cbest = .99
    time_limit = 15

    color_map = mapping.draw_random_map(100)
    pixel_info_map = create_pixel_info_map(color_map)
    
    if( not mapping.point_is_valid(color_map=color_map, coordinates=START_POINT)):
        print("invalid starting point")
        exit()

    if( not mapping.point_is_valid(color_map=color_map, coordinates=GOAL_POINT)):
        print("invalid goal point")
        exit()
    
    print('Starting exploration...')

    starting_node = {"c2c": 0, "parentCoordinates": None, "selfCoordinates": START_POINT, "obstacle": False}
    explored_nodes_list.append(starting_node)
    
    pixel_info_map[starting_node["selfCoordinates"][1]] [starting_node["selfCoordinates"][0]] = starting_node

    
    # --- Run the algorithm ---------------------------
    solution = explore(pixel_map= pixel_info_map, \
                             explored_nodes= explored_nodes_list, \
                             start_point=START_POINT,\
                             goal_point=GOAL_POINT,\
                             goal_radius=GOAL_RADIUS, \
                             num_of_iterations= NUM_OF_ITERATIONS)
    if solution is not None:
        print("Number of iterations needed to find solution: " + str(len(explored_nodes_list)))
        solution = backtrack(last_node= solution, map_= pixel_info_map)
    else: 
        print("Solution not found after " + str(NUM_OF_ITERATIONS) + " points checked!")
        exit()
    end_time = time.time()
    print("Solution found after %s seconds." % (end_time - start_time))


    #--- Display results ----------------------------

    for i in explored_nodes_list:
        mapping.draw_node(child_coordinates=i["selfCoordinates"], \
                          parent_coordinates=i["parentCoordinates"], \
                          map= color_map, color= mapping.BLUE)

    cv.circle(color_map, GOAL_POINT, radius=GOAL_RADIUS, color=mapping.GRAY, thickness=-1)

    for i in solution:
        mapping.draw_node(child_coordinates=i["selfCoordinates"], \
                          parent_coordinates=i["parentCoordinates"], \
                          map= color_map, color= mapping.RED)
                        
    end_point = solution[-1]
    mapping.draw_node(child_coordinates=i["selfCoordinates"], \
                      parent_coordinates= None, \
                      map= color_map, color= mapping.GREEN)
    cv.imshow('informed RRT* Algorithm', color_map)
    cv.waitKey(0)

    print("Explored_nodes_matrix:", len(explored_nodes_list))
    print()
    
