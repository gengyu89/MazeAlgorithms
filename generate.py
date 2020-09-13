# A maze generator and solver implemented with
# DFS, MST, recursive division, and A* search
# 2020-08-03

import os, shutil
from Maze.algorithms import *

if __name__ == '__main__':
    
    # select an algorithm
    #   0 - preview the grid setting
    #   1 - depth-first search
    #   2 - randomized prim
    #   3 - recursive division
    #   4 - randomized kruskal
    choice = 4
    
    # clean up previous output
    if os.path.isdir(CAPDIR):
        shutil.rmtree(CAPDIR)
    
    # clean up the terminal
    os.system("clear")
    mainLoop(choice)

