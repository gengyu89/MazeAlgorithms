# fundamental data structures for implementing algorithms
# 2020-08-03

import time
from Maze.base import *
from copy import deepcopy

PYGAMEWIDTH  = 360  # default: 800
PYGAMEHEIGHT = 360  # default: 500
FPS = 17  # determines the speed of the game
CELLSIZE = 20

# other usable sizes:
#   360 by 360 - so that you have exactly
#       18 by 18 grids for testing
#   600 by 400 - size used by wikipedia example
#   800 by 500 - optimum resolution
#       for making a video
#   940 by 520 - used by this youtube video:
#       https://www.youtube.com/watch?v=6kv5HKPB1XU
#   

# remove hard-coded RGB values
color_FOOD = (255, 0, 0)
color_HEAD = (0, 127, 255)
color_SNAKE = (0, 255, 0)  # borrowed from packt book
color_GRIDS = (40, 40, 40)
color_SCORE = (105, 105, 105)  # borrowed from fangxuecong.com
color_BONES = (255, 255, 255)

# parameters for rendering
SIZESQ = 0.5*CELLSIZE  # size of each segment to be rendered
OFFSET = 0.5*SIZESQ    # offset from the center
THICKNESS = math.ceil(0.1*CELLSIZE)
PAUSETIME = int(10e3)  # time for displaying the final frame [ms]

# when the render unit is smaller than 0.5 times of the cell size
# you have to construct more than one connector block in drawChannel()

def inCanvas(vect):
    """Determines whether an instance of vector
    is within the range of the canvas."""
    
    X_inside = (0 < vect.x < PYGAMEWIDTH)
    Y_inside = (0 < vect.y < PYGAMEHEIGHT)
    
    return X_inside and Y_inside

def validNeighbors(vect):
    """Subroutine for enumerating valid neighbors only."""
    
    N_1 = vector(vect.x-CELLSIZE, vect.y)
    N_2 = vector(vect.x+CELLSIZE, vect.y)
    N_3 = vector(vect.x, vect.y-CELLSIZE)
    N_4 = vector(vect.x, vect.y+CELLSIZE)
    
    return list(filter(inCanvas, [N_1, N_2, N_3, N_4]))

class Maze(object):
    """Essentially an undirected graph."""
    
    def __init__(self):
        self.__nodes = []
        self.__walls = []
    
    @property
    def nodes(self):
        return self.__nodes  # deepcopy() removed
    
    @property
    def walls(self):
        return self.__walls  # deepcopy() removed
    
    def InsertNode(self, node):
        if node in self.__nodes:
            print("Node exists!")
        else:
            self.__nodes.append(node)
    
    def InsertWall(self, wall):
        if wall in self.__walls:
            print("Wall exists!")
        else:
            self.__walls.append(wall)
    
    def RemoveWall(self, target):
        if target in self.__walls:
            self.__walls.remove(target)
        else:
            print("Target not exists!")
    
    def isVisited(self, vect):
        # given a vect, check a node
        for node in self.__nodes:
            if node.current == vect:
                return node.visited
        print("Node not found!")
    
    def SetVisited(self, vect):
        # given a vect, operate on a node
        marked = False
        for i, node in enumerate(self.__nodes):
            if node.current == vect:
                self.__nodes[i].visited = True
                marked = True
        if marked is False:
            # do not call self.hasNode(vect)
            # for repeated traverse
            print("Node not found!")
    
    def hasNode(self, vect):
        # given a vect, test the existence of a node
        vects = [node.current for node in self.nodes]
        return vect in vects
    
    def GetNode(self, vect):
        # given a vect, return the node instance
        for node in self.__nodes:
            if node.current == vect:
                return node  # for statement creates a copy
        print("Node not found!")
    
    def GetNeighbors(self, target):
        # enumerates a cell's connected neighbors
        for node in self.__nodes:
            if node.current == target:
                return node.neighbors
        print("Node not found!")
    
    def SetConnected(self, vect_1, vect_2):
        # given vects, operate on nodes
        assert self.hasNode(vect_1) and self.hasNode(vect_2), \
            "At least one not found!"
        for i, node in enumerate(self.__nodes):
            if node.current == vect_1:
                self.__nodes[i].AddNeighbor(vect_2)
            elif node.current == vect_2:
                self.__nodes[i].AddNeighbor(vect_1)
    
    def SetApart(self, vect_1, vect_2):
        # given vects, operate on nodes
        assert self.hasNode(vect_1) and self.hasNode(vect_2), \
            "At least one not found!"
        for i, node in enumerate(self.__nodes):
            if node.current == vect_1:
                self.__nodes[i].DelNeighbor(vect_2)
            elif node.current == vect_2:
                self.__nodes[i].DelNeighbor(vect_1)
    
    def __str__(self):
        str_nodes = 'Nodes:\n'
        for node in self.__nodes:
            str_nodes += (str(node) + '\n')
        str_walls = 'Walls:\n'
        for wall in self.__walls:
            str_walls += (str(wall) + '\n')
        return str_nodes + str_walls

class Node(object):
    """Data structure for representing each cell."""
    # visited and neighbors are mutable
    # current is not
    def __init__(self, current, neighbors):
        self.__visited   = False  # by default set it unvisited
        self.__current   = current
        self.__neighbors = neighbors
    
    @property
    def visited(self):
        return self.__visited
    
    @visited.setter
    def visited(self, value):
        self.__visited = value
    
    @property
    def current(self):
        return self.__current  # deepcopy() removed
    
    @property
    def neighbors(self):
        return self.__neighbors  # deepcopy() removed
    
    def AddNeighbor(self, new):
        # new - an instance of vector class
        if new in self.neighbors:
            print("Already exists!")
        else:
            self.__neighbors.append(new)
    
    def DelNeighbor(self, target):
        # target - an instance of vector class
        if target in self.neighbors:
            self.__neighbors.remove(target)
        else:
            print("Neighbor not found!")
    
    def ListDisconnected(self):
        """Enumerates a node's unvisited neighbors."""
        for valid in validNeighbors(self.current):
            if valid not in self.neighbors:
                yield valid
    
    def __eq__(self, other):
        if isinstance(other, Node):
            return self.current == other.current
        else:
            return NotImplemented
    
    def __repr__(self):
        name    = self.current
        content = self.neighbors
        return '{!r}: {!r}'.format(name, content)

class DisjointSet(object):
    """Disjoint set data structure."""
    
    def __init__(self, dset=None):
        """Two ways of initialization."""
        if dset is None:  # not provided
            dset = []
        self.__sets = dset
    
    @property
    def sets(self):
        return self.__sets  # deepcopy() removed
    
    def InsertSet(self, new):
        if new in self.__sets:
            print("Set exists!")
        else:
            self.__sets.append(new)
    
    def LocateSet(self, target):
        """Returns the index of the set TARGET belongs to;
        raises an error to prevent further operation such
        as removing and joining, if TARGET is not found."""
        for i, set_ in enumerate(self.__sets):
            if target in set_:
                return i
        raise Exception("Value not found!")
    
    def RemoveSet(self, value):
        """Deletes the set that contains value."""
        i = self.LocateSet(value)
        del self.__sets[i]
    
    def AreSeparated(self, v_1, v_2):
        """Determines whether two values
        belong to different sets."""
        # i, j = self.LocateSet(v_1), self.LocateSet(v_2)
        i, j = self.QuickFind(v_1, v_2)
        return i != j
    
    def QuickFind(self, v_1, v_2):
        """A better implementation for
        searching two elements."""
        i, j = None, None
        for k, set_ in enumerate(self.__sets):
            if v_1 in set_: i = k
            if v_2 in set_: j = k
            if i is not None and j is not None:
                return i, j
        raise Exception("At least one element is not found!")
    
    def JoinSets(self, v_1, v_2):
        """Joins the two sets v_1 and v_2 belong to."""
        # i, j = self.LocateSet(v_1), self.LocateSet(v_2)
        i, j = self.QuickFind(v_1, v_2)
        if i == j:
            print("Cannot join. Values belong to the same set.")
        else:
            new_set = self.__sets[i].union(self.__sets[j])
            self.InsertSet(new_set)
            self.RemoveSet(v_1)
            self.RemoveSet(v_2)
    
    def __len__(self):
        return len(self.__sets)
    
    def __str__(self):
        output = 'Sets:\n'
        for set_ in self.__sets:
            output += (str(set_) + '\n')
        return output

class Wall(object):
    """Data structure for representing cell divisors."""
    # start and end are instances of the vector class
    # and are not allowed to mutate
    def __init__(self, start, end):
        self.__start = start
        self.__end   = end
    
    @property
    def start(self):
        return self.__start  # deepcopy() removed
    
    @property
    def end(self):
        return self.__end  # deepcopy() removed
    
    def __eq__(self, other):
        if isinstance(other, Wall):
            return self.start == other.start \
                and self.end == other.end
        else:
            return NotImplemented
    
    def __repr__(self):
        return 'Start: {!r} End: {!r}'.format(self.start, self.end)

# ----- Classes that are related to animations -----

class Frame(object):
    """Consists of a vector instance representing
    the operational unit and a Maze instance."""
    def __init__(self, head, maze):
        self.__head = head  # the cell that is leading the generation
        self.__maze = maze  # current status of the graph
    
    @property
    def head(self):
        return self.__head  # deepcopy() removed
    
    @property
    def maze(self):
        return self.__maze  # deepcopy() removed

class Path(object):
    """Data structure for storing each frame
    in the path generation animation."""
    def __init__(self, visited, path):
        self.__visited = visited
        self.__path = path
    
    @property
    def visited(self):
        return self.__visited  # deepcopy() removed
    
    @property
    def path(self):
        return self.__path  # deepcopy() removed

class Timer:
    """A Timer class that can be used
    within a context manager."""
    def __init__(self, func=time.perf_counter):
        self.elapsed = 0.0
        self._func = func
        self._start = None
    
    def start(self):
        if self._start is not None:
            raise RuntimeError('Already started')
        else:
            pass
        self._start = self._func()
    
    def stop(self):
        if self._start is None:
            raise RuntimeError('Not started')
        else:
            pass
        end = self._func()
        self.elapsed += end - self._start
        self._start = None
    
    def reset(self):
        self.elapsed = 0.0
    
    @property
    def running(self):
        return self._start is not None
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, *args):
        self.stop()
        print('Elapsed time is %f seconds.\n' \
            % self.elapsed)

# Here are several ways you can invoke it
# 
# t = Timer()
# with t:
#     # run something
# 
# with Timer() as t:
#     # run something
# 
# You do not even have to
#   provide it a variable name, such as
# 
# with Timer():
#     # run something
# 

