from os import path
import math
# from typing import final
from .Capstone import *
import time


class PathPlanner:
    def __init__(self, end_x, end_y, node_distance, x=1, y=1):
        self.min_x = int(x)
        self.min_y = int(y)
        self.max_x = 5
        self.max_y = 7
        self.beacon_pos=[(0.38,0.01),(3.84,0.01),(1.48,7.59),(2.54,7.59)]
        self.obstacle = [(2,3)]
        self.end_x = int(end_x)
        self.end_y = int(end_y)
        self.node_distance = node_distance
        self.currentPoint = (x, y)
        self.path=[]
        self.route=[]
        self.num_nodes_x = int((end_x - x)/node_distance) +1
        self.num_nodes_y = int((end_y - y)/node_distance) +1
       
        


    #SweepingPath creates the entire grid route for the rover to avoid obstacles
    def sweepingPath(self):
        # start = time.time()
        final_path=[]
        j=0
        while j < self.num_nodes_y:
            i=0
            while i < self.num_nodes_x:
                if j%2==0:
                    self.path.append((self.min_x + i * self.node_distance, self.min_y + j * self.node_distance))
                    i+=1
                else:
                    self.path.append((self.end_x - i * self.node_distance, self.min_y + j * self.node_distance))
                    i+=1
            j+=1
        
        for point in self.path:
            if point in self.obstacle:
                index=self.path.index(point)
                final_path += self.shortestPath(self.path[index-1], self.path[index+1])[1:-1]
            else:
                final_path.append(point)

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

        currentPoint =  ( (currentPoint[0] - self.min_x)/self.node_distance, (currentPoint[1]-self.min_y)/self.node_distance )
        targetPoint =  ( (targetPoint[0] - self.min_x)/self.node_distance, (targetPoint[1]-self.min_y)/self.node_distance )
        grid = GridWithWeights(self.num_nodes_x, self.num_nodes_y)
        wall = []
        for g in self.obstacle:
            wall.append(((g[0] - self.min_x)/self.node_distance,(g[1]-self.min_y)/self.node_distance) )
        grid.walls = wall
        came_from_bfs = breadth_first_search(grid, currentPoint, targetPoint)
        x = reconstruct_path(came_from_bfs, currentPoint, targetPoint)
        y=[]
        for i in x:
            y.append((i[0] * self.node_distance + self.min_x, i[1] * self.node_distance + self.min_y))
        return y

    def update(self,targetPoint): #working on this now
        self.route = self.shortestPath(self.currentPoint,targetPoint) + self.state(self.currentPoint)[1]

    #Obtain the nearest node to the current Point
    def nearestPoint(self, currentPoint):
        diff=[]
        for i in range(len(self.path)):
            diff.append(abs(self.path[i][0]-currentPoint[0]) + abs(self.path[i][1]-currentPoint[1]))
        return (self.path[diff.index(min(diff))])






'''The 2 sets of code below shows you ROUGHLY how your code will be used

Finding red object

pp = PathPlanner()
arrayOfPoints = pp.shortestPath((1,1), (4,4)) 

for point in arrayOfPoints:
    robot.move_to( Point(x=point[0], y=point[1]) )



Sweeping

pp = PathPlanner()
arrayOfPoints = pp.coveragePath( (1,1) ) 

for point in arrayOfPoints:
    robot.move_to( Point(x=point[0], y=point[1]) )
'''