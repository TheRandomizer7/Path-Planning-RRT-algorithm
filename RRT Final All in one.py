import matplotlib.pyplot as plt
import random
import math
from shapely.geometry import Point, Polygon, LineString

#Functions to be used
def CheckIfValidPoint(point, node_to_connect, obstacles):
    is_valid = True
    #Check if the point is within any obstacle or not
    for obj in obstacles:
        if(obj.polygon.contains(Point(point[0], point[1]))):
            is_valid = False
            break
    
    #Check if after joining the point with node, will the line pass through any objects or not
        for j in range(len(obj.coord_list) - 1):
            line_a = LineString([point, (node_to_connect.x_coord, node_to_connect.y_coord)])
            line_b = LineString([(obj.coord_list[j][0], obj.coord_list[j][1]), (obj.coord_list[j+1][0], obj.coord_list[j+1][1])])
            if(line_a.intersects(line_b)):
                is_valid = False
                break

    return is_valid

def AddPointToTree(rand_point, min_index, nodes_in_tree, goal):
    found_goal = False

    nodes_in_tree.append(Nodes(rand_point[0], rand_point[1], min_index))

    if(goal.polygon.contains(Point(rand_point[0], rand_point[1]))):
        found_goal = True

    return found_goal

def SamplePoint(counter, bounds_of_plane, goal, sample_goal):
    #sample random point
    rand_point = [random.uniform(0, bounds_of_plane[0]), random.uniform(0, bounds_of_plane[1])]

    #sample every 5th point in goal region
    if(counter % sample_goal == 0):
        min_x, min_y, max_x, max_y = goal.polygon.bounds
        rand_point = [random.uniform(min_x, max_x), random.uniform(min_y, max_y)]
    
    return rand_point

#Classes
class Figures:

    def __init__(self, coord_list):
        self.coord_list = coord_list
        self.polygon = Polygon(coord_list)

class Nodes:

    def __init__(self, x_coord, y_coord, parent_index):
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.parent_index = parent_index

def RRT(start, goal_point, obstacle_list):
    #Variables
    bounds_of_plane = (10, 10)
    fixed_distance = 0.25 #radius in which the next node should lie 
    sample_goal = 5 #Every 5th point is sampled in the goal region
    counter = 0 #just keeping a count the number of loops, so as to sample every 5th point in the goal region
    found_goal = False

    #initializing obstacles
    obstacles = []
    for obst in obstacle_list:
        obst.append(obst[0])
        obstacles.append(Figures(obst))

    #initializing goal region (square of side 0.2, goal point is a corner of the goal region)
    goal = Figures([(goal_point[0], goal_point[1]), (goal_point[0], goal_point[1] - 0.2), (goal_point[0] - 0.2, goal_point[1] - 0.2), (goal_point[0] - 0.2, goal_point[1])])
    
    #initializing tree with start point
    nodes_in_tree = []
    nodes_in_tree.append(Nodes(start[0], start[1], 0))

    #Main loop of RRT
    while (not found_goal):
        #Sampling point
        rand_point = SamplePoint(counter, bounds_of_plane, goal, sample_goal)
        counter = counter + 1
        
        #Calculating the point in the tree which has minimum distance from sampled point
        min_dist = math.sqrt((rand_point[0] - start[0]) ** 2 + (rand_point[1] - start[1]) ** 2)
        min_index = 0
    
        for i in range(len(nodes_in_tree)):
            curr_dist = math.sqrt((rand_point[0] - nodes_in_tree[i].x_coord) ** 2 + (rand_point[1] - nodes_in_tree[i].y_coord) ** 2)
            if (curr_dist < min_dist):
                min_dist = curr_dist
                min_index = i
        
        #Changing point if the point has a distance greater than fixed_distance from the nearest node
        if(min_dist > fixed_distance):
            rand_point[0] = (((rand_point[0] - nodes_in_tree[min_index].x_coord) * fixed_distance)/min_dist) + nodes_in_tree[min_index].x_coord
            rand_point[1] =  (((rand_point[1] - nodes_in_tree[min_index].y_coord) * fixed_distance)/min_dist) + nodes_in_tree[min_index].y_coord
        
        #Check if the final calculated point is valid
        if(not CheckIfValidPoint(rand_point, nodes_in_tree[min_index], obstacles)):
            continue
        
        #Add valid point in tree and check if goal is found or not
        found_goal = AddPointToTree(rand_point, min_index, nodes_in_tree, goal)

        #if the goal has been found, add the final goal point to the tree
        if(found_goal):
            AddPointToTree(goal_point, len(nodes_in_tree) - 1, nodes_in_tree, goal)

    return nodes_in_tree

def visualize(nodes_in_tree, obstacle_list):
    #Drawing obstacles
    for obst in obstacle_list:
        obst.append(obst[0])
        xs, ys = zip(*obst)
        plt.plot(xs, ys)

    #Plotting and connecting nodes with respective parent nodes in tree
    for node in nodes_in_tree:
        plt.plot([node.x_coord, nodes_in_tree[node.parent_index].x_coord], [node.y_coord, nodes_in_tree[node.parent_index].y_coord], "r.-", markersize = 3, linewidth = 0.3)

    #Connecting Goal to the start point
    curr_index = len(nodes_in_tree) - 1
    while(curr_index != 0):
        parent_index = nodes_in_tree[curr_index].parent_index
        plt.plot([nodes_in_tree[curr_index].x_coord, nodes_in_tree[parent_index].x_coord], [nodes_in_tree[curr_index].y_coord, nodes_in_tree[parent_index].y_coord], 'b.-', markersize = 5, linewidth = 0.5)
        curr_index = parent_index

    plt.show()

def test_rrt():
    
    obstacle_list = [
            [(2, 10), (7, 10), (6, 7), (4, 7), (4, 9), (2, 9)],
            [(3, 1), (3, 6), (4, 6), (4, 1)],
            [(7, 3), (7, 8), (9, 8), (9, 3)],
        ]

    start = (1, 1)
    goal = (10, 10)
    
    path = RRT(start,goal, obstacle_list)
     
    visualize(path, obstacle_list)

test_rrt()
