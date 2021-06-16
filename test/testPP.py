#! /usr/bin/env python3
from grid_path_planner.PathPlanner import PathPlanner


#User Guide
pp=PathPlanner(end_x=3, end_y=6, node_distance = 0.25, x=1, y=4) #initiate class
pp.sweepingPath() #Create grid pathing route for rover (List of tuples)
# pp.currentPoint= ((2, 2)) #Update current Position
# pp.state(pp.currentPoint)  #Get the state of completion 
# print(pp.path)
print(pp.nearestPoint((2.1,4.1)))



# print(pp.nearestPoint(pp.currentPoint))