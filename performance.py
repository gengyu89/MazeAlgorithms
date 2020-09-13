# some performance tests for creating instances
# created by: Yu Geng
# 2020-08-06

import random
from Maze.base import *
from Maze.structs import *

X_GRID = range(0, PYGAMEWIDTH, CELLSIZE)
Y_GRID = range(0, PYGAMEHEIGHT, CELLSIZE)

# to mimic what I am doing in the game, I assume vector sizes
# are not known and pre-allocating for performance is not feasible

# create a hundred vector instances
print("Vectors:")
vectors = []

with Timer() as t:
    for _ in range(100):
        v = vector(random.choice(X_GRID), random.choice(Y_GRID))
        vectors.append(v)

# create a hundred dictionaries
print("Dictionaries:")
dicts = []

with Timer() as t:
    for _ in range(100):
        d = {'x': random.choice(X_GRID), 'y': random.choice(Y_GRID)}
        dicts.append(d)

# create a hundred two-element tuples
print("Tuples:")
tuples = []

with Timer() as t:
    for _ in range(100):
        t = (random.choice(X_GRID), random.choice(Y_GRID))
        tuples.append(t)

