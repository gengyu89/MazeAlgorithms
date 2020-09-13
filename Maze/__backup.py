# discarded codes go here
# 2020-08-06

# ----- Subroutines for rendering objects -----

def drawMazeNodes(frame):
    """Naive way of rendering the current status of a maze
    by drawing channels between connected cells."""
    # in this case, frame is a Frame instance
    # rather than a graph
    
    # render the main body
    for node in frame.maze.nodes:
        for neighbor in node.neighbors:
            drawChannel(node.current, neighbor)
    
    # render the construction cell
    X, Y = frame.head.x, frame.head.y
    head_inner_rect = pygame.Rect(X-OFFSET, Y-OFFSET, SIZESQ, SIZESQ)
    pygame.draw.rect(screen, color_FOOD, head_inner_rect)

def drawMazeWalls(maze):
    """Naive way of rendering the current status of a maze
    according to the connectivities between nodes."""
    # be aware that this will result in
    # many duplicated renders
    
    for node in maze.nodes:
        # enumerate all valid neighbors and
        # examine their connectivities with node
        for neighbor in validNeighbors(node.current):
            if neighbor not in node.neighbors:
                drawWall(node.current, neighbor)

def drawWalls(maze):
    """Renders the current status of a maze
    directly from the walls attribute."""
    
    for wall in maze.walls:
        coord_0 = (wall.start.x, wall.start.y)
        coord_1 = (wall.end.x, wall.end.y)
        pygame.draw.line(screen, color_BONES, coord_0, coord_1, THICKNESS)

# ----- Subroutines for controlling gameflow -----

def createGraph(connected):
    """Subroutine for creating a graph."""
    # connected=True - start with no walls
    # connected=False - start with walls
    
    print("Creating graph...")
    graph = Maze()
    
    for X, Y in product(X_grid, Y_grid):
        cell_x, cell_y = X + CELLSIZE//2, Y + CELLSIZE//2
        current = vector(cell_x, cell_y)
        if connected:
            neighbors = validNeighbors(current)
        else:
            neighbors = []
        graph.InsertNode(Node(current, neighbors))
    
    print('Window size: %d*%d' % (PYGAMEWIDTH, PYGAMEHEIGHT))
    print("")
    
    return graph

def createFullGraph():
    """Creates a graph with both nodes and walls."""
    # in this case, no parameter is passed because
    # adjacent nodes can be only disconnected
    
    print("Creating graph...")
    graph = Maze()
    
    N_cells, N_walls = 0, 0
    for X, Y in product(X_grid, Y_grid):
        
        # create and insert a node
        cell_x, cell_y = X + CELLSIZE//2, Y + CELLSIZE//2
        current = vector(cell_x, cell_y)
        graph.InsertNode(Node(current, []))
        N_cells += 1
        
        # create and insert walls
        for wall in makeWalls(X, Y):
            graph.InsertWall(wall)
            N_walls += 1
    
    print('%d cells and %d walls created.' % (N_cells, N_walls))
    print("")
    
    return graph

# the following implementation is dangerous
# because an unvisited neighbor could be traversed through
# and marked as visited at some deeper depth of the recursion (a U-turn)
# as a result, you will see the following error message:
#   unvisited.remove(new_start)
# ValueError: list.remove(x): x not in list

def dfs_old(start, graph, unvisited, frames=[]):
    """Subroutine for traversing a maze
    and marking each passing node as visited."""
    
    if unvisited == []:  # base case triggered
        return frames
    
    # enumerate unvisited neighbors
    unvisitedNeighbors = \
        [N for N in validNeighbors(start) \
        if not graph.isVisited(N)]
    
    if unvisitedNeighbors == []:  # dead end
        return False
    
    random.shuffle(unvisitedNeighbors)
    for new_start in unvisitedNeighbors:
            graph.SetConnected(start, new_start)
            graph.SetVisited(new_start)
            unvisited.remove(new_start)
            frames.append(Frame(new_start, deepcopy(graph)))
            isFull = dfs(new_start, graph, unvisited, frames)
            if isFull:
                return isFull
            else:
                continue
    
    # after trying all routes,
    # the graph is still not filled
    return False  # backtrack to the upper junction

