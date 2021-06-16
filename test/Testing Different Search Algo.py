from os import path
import math
from typing import final
from Capstone import *
import time


class PathPlanner:
    def __init__(self, end_x, end_y, node_distance, x=1, y=1):
        self.min_x = x
        self.min_y = y
        self.max_x = 50000
        self.max_y = 70000
        self.beacon_pos=[(0.38,0.01),(3.84,0.01),(1.48,7.59),(2.54,7.59)]
        self.obstacle = [(2,3)]
        self.end_x = end_x
        self.end_y = end_y
        self.node_distance = node_distance
        self.currentPoint = (x, y)
        self.path=[]
        self.route=[]
        self.num_nodes_x = math.floor(end_x - x)/node_distance
        self.num_nodes_y = math.floor(end_y - y)/node_distance
       
        


    #SweepingPath creates the entire grid route for the rover to avoid obstacles
    def sweepingPath(self):
        # start = time.time()
        final_path=[]
        j=0
        while j <= self.num_nodes_y:
            i=0
            while i <= self.num_nodes_x:
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

        # self.path = [index for index in self.path if index not in self.obstacle] #Remove routes from walls
        # index=0
        # while index < len(self.path)-1: 
        #     final_path += self.shortestPath(self.path[index], self.path[index+1])[:-1]
        #     index+=1
        # final_path += [self.path[-1]]
        # end= time.time()
        self.path = final_path
        # print(end-start)
    
    #Show the data of the progress of the sweep
    def state(self,currentPoint): 
        covered = self.path[:self.path.index(self.nearestPoint(currentPoint))+1]
        uncovered = self.path[self.path.index(self.nearestPoint(currentPoint))+1:]
        percentage_complete = str(round(len(covered)/len(covered+uncovered),2)) + '%'
        return covered, uncovered, percentage_complete

    #Obtain the list of tuple to bring the rover on the shortest path
    def shortestPath(self, currentPoint, targetPoint): #functionName(inputVariableName : InputType) -> returnType:
        
        grid = GridWithWeights(self.end_x+1, self.end_y+1)
        grid.walls=self.obstacle

        start_dij=time.time()
        came_from_dij = dijkstra_search(grid, currentPoint, targetPoint)
        reconstruct_path(came_from_dij, currentPoint, targetPoint)
        end_dij=time.time()

        start_bfs=time.time()
        came_from_bfs = breadth_first_search(grid, currentPoint, targetPoint)
        reconstruct_path(came_from_bfs, currentPoint, targetPoint)
        end_bfs=time.time()


        start_ast=time.time()
        came_from_ast = a_star_search(grid, currentPoint, targetPoint)
        reconstruct_path(came_from_ast, currentPoint, targetPoint)
        end_ast=time.time()
        

        return round((end_dij-start_dij),4), round((end_bfs-start_bfs),4), round((end_ast-start_ast),4)

    def update(self,targetPoint): #working on this now
        self.route = self.shortestPath(self.currentPoint,targetPoint) + self.state(self.currentPoint)[1]

    #Obtain the nearest node to the current Point
    def nearestPoint(self,currentPoint):
        diff=[]
        for i in range(len(self.path)):
            diff.append(abs(self.path[i][0]-currentPoint[0]) + abs(self.path[i][1]-currentPoint[1]))
        return(self.path[diff.index(min(diff))])


#User Guide
# pp=PathPlanner(end_x=50, end_y=50, node_distance=1) #initiate class
# pp.sweepingPath() #Create grid pathing route for rover (List of tuples)
# pp.currentPoint= (2,2) #Update current Position
# pp.state(pp.currentPoint)  #Get the state of completion 
# pp.update()

n = list(range(50,1000,50))
dij=[]
bfs=[]
ast=[]
for i in n:
    pp=PathPlanner(end_x=i, end_y=i, node_distance=1)
    r = pp.shortestPath((2,2),(pp.end_x, pp.end_y))
    dij.append(r[0])
    bfs.append(r[1])
    ast.append(r[2])

print(dij)
print(bfs)
print(ast)

