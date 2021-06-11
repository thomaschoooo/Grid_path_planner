# Sample code from https://www.redblobgames.com/pathfinding/a-star/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

from __future__ import print_function
# some of these types are deprecated: https://www.python.org/dev/peps/pep-0585/
from typing import Protocol, Dict, List, Iterator, Tuple, TypeVar, Optional
T = TypeVar('T')
import heapq
import collections
Location = TypeVar('Location')
GridLocation = Tuple[int, int]

#Setup for Graph
class Graph(Protocol):
    def neighbors(self, id): 
        # type (Location) -> List[Location]
        pass

class WeightedGraph(Graph):
    def cost(self, from_id, to_id): 
        # type (Location, Location) -> float
        pass

class SquareGrid(object):
    def __init__(self, width, height):
        # type (int, int) -> SquareGrid
        self.width = width
        self.height = height
        self.walls = [] # type (List[GridLocation])
    
    def in_bounds(self, id):
        # type (GridLocation) -> bool
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        # type (GridLocation) -> bool
        return id not in self.walls
    
    def neighbors(self, id):
        # type (GridLocation) -> Iterator[GridLocation]
        (x, y) = id
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)] # E W N S
        # see "Ugly paths" section for an explanation:
        if (x + y) % 2 == 0: neighbors.reverse() # S N W E
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results

class GridWithWeights(SquareGrid):
    def __init__(self, width, height):
        # type (int, int) -> GridWithWeights
        super(GridWithWeights, self).__init__(width, height)
        self.weights = {} # type(Dict[GridLocation, float])
    
    def cost(self, from_node, to_node):
        # type (GridLocation, GridLocation) -> float
        return self.weights.get(to_node, 1)


# Grahpics of Graph
def from_id_width(id, width):
    return (id % width, id // width)

def draw_tile(graph, id, style):
    r = " . "
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if 'path' in style and id in style['path']:   r = " @ "
    if 'start' in style and id == style['start']: r = " S "
    if 'goal' in style and id == style['goal']:   r = " G "
    if id in graph.walls: r = "###"
    return r

def draw_grid(graph, **style):
    print("___" * graph.width)
    for y in range(graph.height):
        for x in range(graph.width):
            print("%s" % draw_tile(graph, (x, y), style), end="")
        print()
    print("~~~" * graph.width)



#Setup for Queue
class PriorityQueue:
    def __init__(self):
        self.elements = [] # type (List[Tuple[float, T]])
    
    def empty(self):
        # type () -> bool
        return not self.elements
    
    def put(self, item, priority):
        # type (T, float) -> None
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        # type () -> T
        return heapq.heappop(self.elements)[1]




##Change these parameters
l=30 #Number of Nodes on length
b=17 #Number of Nodes on Breadth

#Algo for snakey movement
x=[]
n=0

switch = 0
while n<=b:
    if n%2==0:
        n+=1
        
    else:
        if switch == 0:
            for j in range(0,l-1):
                x.append((j,n))
            switch = 1
            n+=1
        else:
            for j in range(1,l):
                x.append((j,n))
            switch = 0
            n+=1


#Create the Grid with our parameters above
diagram4 = GridWithWeights(l, b)
#diagram4.walls = x #i.e. cannot pass through
diagram4.weights = {loc: 5*l for loc in x} #Set the weights of the nodes listed in my algo above
start, goal = (0,0), (l-1,b-1) #start at top left end at bottom right



#Dijkstra Optimisation (Path Search)
def dijkstra_search(graph, start, goal):
    # type (WeightedGraph, Location, Location) -> 
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {} # type (Dict[Location, Optional[Location]])
    cost_so_far = {} # type (Dict[Location, float])
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get() # type (Location)
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far


#Shows the optimised path with @
def reconstruct_path(came_from, start, goal):
    # type (Dict[Location, Location], Location, Location) -> List[Location]
    current = goal # type (Location)
    path = [] # type (List[Location])
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path


#Run Code
came_from, cost_so_far = dijkstra_search(diagram4, start, goal)
draw_grid(diagram4, number=cost_so_far, start=start, goal=goal) #Weight for each node from the start
draw_grid(diagram4, path=reconstruct_path(came_from, start=start, goal=goal)) #Drawing optimal path

