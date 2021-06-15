from os import path
import math
# from typing import final
from .Capstone import *

#Fixed Room Parameters 
max_x = 5
max_y = 7
min_x = 1
min_y = 1
beacon_pos=[(0.38,0.01),(3.84,0.01),(1.48,7.59),(2.54,7.59)]
obstacle = []#[(2,3)] 

#Grid formation
grid = GridWithWeights(max_x, max_y)
grid.walls = obstacle

class PathPlanner:
    def __init__(self, end_x, end_y, node_distance, x=min_x, y=min_y):
        self.end_x = end_x
        self.end_y = end_y
        self.node_distance = node_distance
        self.currentPoint = (x, y)
        self.path=[]
        self.route=[]
        self.num_nodes_x = math.floor(end_x - min_x)/node_distance
        self.num_nodes_y = math.floor(end_y - min_y)/node_distance

    #SweepingPath creates the entire grid route for the rover to avoid obstacles
    def sweepingPath(self):
        grid = GridWithWeights(self.end_x, self.end_y)
        final_path=[]
        j=0
        while j <= self.num_nodes_y:
            i=0
            while i <= self.num_nodes_x:
                if j%2==0:
                    self.path.append((min_x + i * self.node_distance, min_y + j * self.node_distance))
                    i+=1
                else:
                    self.path.append((self.end_x - i * self.node_distance, min_y + j * self.node_distance))
                    i+=1
            j+=1
        self.path = [index for index in self.path if index not in obstacle] #Remove routes from walls
        index=0
        while index < len(self.path)-1: 
            final_path += self.shortestPath(self.path[index], self.path[index+1])[:-1]
            grid.weights[self.path[index]] = 10
            index+=1
        final_path += [self.path[-1]]
        self.path = final_path
        self.route = self.path
    
    #Show the data of the progress of the sweep
    def state(self,currentPoint): 
        covered = self.path[:self.path.index(self.nearestPoint(currentPoint))+1]
        uncovered = self.path[self.path.index(self.nearestPoint(currentPoint))+1:]
        percentage_complete = str(round(len(covered)/len(covered+uncovered),2)) + '%'
        return covered, uncovered, percentage_complete

    #Obtain the list of tuple to bring the rover on the shortest path
    def shortestPath(self, currentPoint, targetPoint): #functionName(inputVariableName : InputType) -> returnType:
        came_from = dijkstra_search(grid, currentPoint, targetPoint) 
        return reconstruct_path(came_from, currentPoint, targetPoint)

    def update(self,targetPoint): #working on this now
        self.route = self.shortestPath(self.currentPoint,targetPoint) + self.state(self.currentPoint)[1]

    #Obtain the nearest node to the current Point
    def nearestPoint(self,currentPoint):
        diff=[]
        for i in range(len(self.path)):
            diff.append(abs(self.path[i][0]-currentPoint[0]) + abs(self.path[i][1]-currentPoint[1]))
        return(self.path[diff.index(min(diff))])


# pp = PathPlanner(4, 4, 1)
# pp.sweepingPath() #Create grid pathing route for rover (List of tuples)
# pp.currentPoint=(2,2) #Update current Position
# pp.state(pp.currentPoint)  #Get the state of completion 
# print(pp.state(pp.currentPoint))
# print("Route: ", end=" ")
# print(pp.route)
