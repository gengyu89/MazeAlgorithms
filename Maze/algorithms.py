# subroutines for implementing maze generators
# 2020-08-03

import sys, math, random, pygame

from Maze.structs import *
from pygame.locals import *
from itertools import product

# check validity before continuing
assert PYGAMEWIDTH  % CELLSIZE == 0, "Width is not divisible!"
assert PYGAMEHEIGHT % CELLSIZE == 0, "Height is not divisible!"
X_GRID = range(0, PYGAMEWIDTH, CELLSIZE)
Y_GRID = range(0, PYGAMEHEIGHT, CELLSIZE)

# parameters for the canvas
MATRIX_W = PYGAMEWIDTH//CELLSIZE
MATRIX_H = PYGAMEHEIGHT//CELLSIZE
MATRIX_SIZE = MATRIX_W * MATRIX_H
BGCOLOR = (0, 0, 0)

# algorithms
CAPDIR = './capture/'
ALGFUN = {0: "viewSetting()", \
          1: "runDFS()", 2: "runPrim()", \
          3: "runDivide()", 4: "runKruskal()"}
ALGTIT = {0: 'Grid Setting (%d x %d Cells)' % (MATRIX_W, MATRIX_H), \
          1: "Depth-First Search", 2: "Randomized Prim", \
          3: "Recursive Division", 4: "Randomized Kruskal"}

# ----- Subroutines for rendering objects -----

def drawGrid():
    """Subroutine for displaying
    the grid setting - Method I."""
    
    for X in X_GRID:
        pygame.draw.line(screen, color_GRIDS, (X, 0), (X, PYGAMEHEIGHT))
    for Y in Y_GRID:
        pygame.draw.line(screen, color_GRIDS, (0, Y), (PYGAMEWIDTH, Y))

def drawCenters():
    """Subroutine for displaying
    the grid setting - Method II."""
    
    radius = int(0.1 * CELLSIZE)
    for X, Y in product(X_GRID, Y_GRID):
        x, y = X + CELLSIZE//2, Y + CELLSIZE//2
        pygame.draw.circle(screen, color_GRIDS, (int(x), int(y)), radius)

def drawChannel(cell_1, cell_2, color=color_BONES):
    """Displays a channel connects two cells
    defined by cell_1 and cell_2."""
    
    # construct rendering arrays
    X_render = [cell_1.x, 0.5 * (cell_1.x+cell_2.x), cell_2.x]
    Y_render = [cell_1.y, 0.5 * (cell_1.y+cell_2.y), cell_2.y]
    
    for X, Y in zip(X_render, Y_render):
        inner_rect = pygame.Rect(X-OFFSET, Y-OFFSET, SIZESQ, SIZESQ)
        pygame.draw.rect(screen, color, inner_rect)

def drawWall(cell_1, cell_2):
    """Displays a wall between two cells
    defined by cell_1 and cell_2."""
    
    if cell_1.x == cell_2.x:  # up-and-down relationship
        X_0 = cell_1.x - 0.5*CELLSIZE
        X_1 = cell_1.x + 0.5*CELLSIZE
        Y_0 = 0.5 * (cell_1.y + cell_2.y)
        Y_1 = Y_0
    elif cell_1.y == cell_2.y:  # left-and-right relationship
        Y_0 = cell_1.y - 0.5*CELLSIZE
        Y_1 = cell_1.y + 0.5*CELLSIZE
        X_0 = 0.5 * (cell_1.x + cell_2.x)
        X_1 = X_0
    
    pygame.draw.line(screen, color_BONES, \
        (X_0, Y_0), (X_1, Y_1), THICKNESS)

def isStaggered(node):
    """Determines whether a node has
    different row and column parities."""
    
    # obtain grid coordinate (the ll corner)
    cell_x, cell_y = node.current.x, node.current.y
    grid_x, grid_y = cell_x - CELLSIZE//2, cell_y - CELLSIZE//2
    
    # locate them from the grid arrays
    index_x, index_y = X_GRID.index(grid_x), Y_GRID.index(grid_y)
    
    # fortunately, range objects support
    # index() like lists do
    
    return (index_x + index_y) % 2

def drawMazeNodes(frame):
    """Smarter way of rendering the current status of a maze
    by drawing channels between connected cells."""
    
    # enumerate nodes and filter out
    # the ones to be rendered
    nodes = (node for node in frame.maze.nodes if isStaggered(node))
    
    # render the main body
    for node in nodes:
        for neighbor in node.neighbors:
            drawChannel(node.current, neighbor)
    
    # render the construction cell
    X, Y = frame.head.x, frame.head.y
    head_inner_rect = pygame.Rect(X-OFFSET, Y-OFFSET, SIZESQ, SIZESQ)
    pygame.draw.rect(screen, color_SNAKE, head_inner_rect)

def drawMazeWalls(maze):
    """Smarter way of rendering the current status of a maze
    according to the connectivities between nodes."""
    
    # enumerate nodes and filter out
    # the ones to be rendered
    nodes = (node for node in maze.nodes if isStaggered(node))
    
    # enumerate all valid neighbors and
    # examine their connectivities with node
    for node in nodes:
        for neighbor in node.ListDisconnected():  # which will return a generator
            drawWall(node.current, neighbor)

def drawPath(path_):
    """Subroutine for rendering visited
    and shortest path cells all together."""
    # path_ - an instance of the Path class
    
    # unpack the Path instance
    visited, path = path_.visited, path_.path
    
    # render visited cells
    # offset, sizesq = 0.4*CELLSIZE, 0.8*CELLSIZE
    for cell in visited:
        X, Y = cell.x, cell.y
        # head_inner_rect = pygame.Rect(X-offset, Y-offset, sizesq, sizesq)
        head_inner_rect = pygame.Rect(X-OFFSET, Y-OFFSET, SIZESQ, SIZESQ)
        pygame.draw.rect(screen, color_SCORE, head_inner_rect)
    
    # determine workflow
    N = len(path)
    if N == 0:
        return  # do not execute the rest of the function
    
    # render the shortest path
    if N == 1:
        X, Y = path[0].x, path[0].y
        inner_rect = pygame.Rect(X-OFFSET, Y-OFFSET, SIZESQ, SIZESQ)
        pygame.draw.rect(screen, color_HEAD, inner_rect)
    else:
        for i in range(1, N):
            prev, curr = path[i-1], path[i]
            drawChannel(prev, curr, color_HEAD)

# ----- Subroutines for controlling gameflow -----

def closeGame():
    """Closes the game panel."""    
    pygame.quit()
    sys.exit()

def mainLoop(choice):
    """Subroutine for running all algorithms."""
    # initialize
    resolution = (PYGAMEWIDTH, PYGAMEHEIGHT)
    flags = DOUBLEBUF  # do not use NOFRAME
    global screen, clock
    
    pygame.init()
    screen = pygame.display.set_mode(resolution, flags)
    pygame.display.set_caption(ALGTIT[choice])
    clock = pygame.time.Clock()
    
    while True:
        os.mkdir(CAPDIR)
        last_frame = eval(ALGFUN[choice])
        
        if last_frame is not None:
            showStatus("Using A* Search...", False)
            runAstar(last_frame)
            showStatus("Solved!", True)
        
        pygame.time.delay(PAUSETIME)
        closeGame()


def createGraph(connected, walls=False):
    """Subroutine for creating a graph."""
    # connected=True - start with no walls
    # connected=False - start with walls
    
    print("Creating graph...")
    graph = Maze()
    
    N_cells, N_walls = 0, 0
    for X, Y in product(X_GRID, Y_GRID):
        
        # create and insert a node
        cell_x, cell_y = X + CELLSIZE//2, Y + CELLSIZE//2
        current = vector(cell_x, cell_y)
        neighbors = validNeighbors(current) if connected else []
        graph.InsertNode(Node(current, neighbors))
        N_cells += 1
        
        # create and insert walls
        for wall in makeWalls(X, Y):
            graph.InsertWall(wall)
            N_walls += 1
    
    if walls:
        print('%d cells and %d walls created.\n' % (N_cells, N_walls))
    else:
        print('Window size: %d*%d\n' % (PYGAMEWIDTH, PYGAMEHEIGHT))
    
    return graph

def viewSetting(method=1):
    """Subroutine for displaying grids
    without running algorithms."""
    
    assert isinstance(method, int), \
        "Please provide an integer."
    
    cur = 0  # initialize frame
    calldict = {1: "drawGrid()", 2: "drawCenters()"}
    
    print('Playing back at %d fps...' % FPS)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                closeGame()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    closeGame()
        
        status = '\rCurrent frame: %d/%d' % (cur+1, FPS)
        sys.stdout.write(status)
        sys.stdout.flush()
        
        screen.fill(BGCOLOR)
        exec(calldict[method])
        
        pygame.display.update()
        clock.tick(FPS)
        
        cur += 1  # update frame
        if cur == FPS:  # show exactly one second
            print('\n')
            return
        
def showStatus(message, hold):
    """Subroutine for displaying a message."""
    
    # Menlo.ttc and Monaco.dfont both look ugly in this project
    # Tahoma.ttf is a much smaller file than PingFang.ttc
    
    # construct a text box
    center_x, center_y = PYGAMEWIDTH//2, PYGAMEHEIGHT//2
    title_font = pygame.font.Font('Tahoma.ttf', 18)
    title_text = title_font.render(message, True, color_BONES)
    text_rect = title_text.get_rect()
    text_rect.centerx = center_x
    text_rect.centery = center_y
    
    # render a rectangle
    offset_x, offset_y = 0.5*text_rect.width, 0.5*text_rect.height
    text_box_rect = pygame.Rect(center_x - offset_x - 5, center_y - offset_y, \
        text_rect.width + 10, text_rect.height)
    
    # render the text
    pygame.draw.rect(screen, color_GRIDS, text_box_rect)
    screen.blit(title_text, text_rect)
    pygame.display.update()
    
    # capture the screen
    cap = 'path' if hold else 'maze'
    pygame.image.save(screen, './capture/%s_final.tga' % cap)
    
    # hold the interface
    while hold:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or \
                (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                closeGame()
    
    # another solution is
    # pygame.time.delay(PAUSETIME)
    # closeGame()

# ----- Depth-First Search -----

def dfs(start, graph, unvisited, frames=[]):
    """Subroutine for traversing a maze recursively
    and marking each accessed node as visited."""
    
    if unvisited == []:  # base case triggered
        return frames
    
    # enumerate all neighbors
    neighbors = [N for N in validNeighbors(start)]
    random.shuffle(neighbors)
    
    for new_start in neighbors:
        if graph.isVisited(new_start):
            continue
        else:
            graph.SetConnected(start, new_start)
            graph.SetVisited(new_start)
            unvisited.remove(new_start)
            frames.append(Frame(new_start, deepcopy(graph)))
            isFull = dfs(new_start, graph, unvisited, frames)
            if isFull:
                return isFull
            else:
                continue
    
    # if this statement is reached, that means
    # either there are no unvisited neighbors (dead end)
    # or all routes starting from the current node have been tried
    # but the maze is still not filled
    return False

def runDFS():
    """Subroutine for creating a graph
    and invoking depth-first search."""
    
    # create a graph and enumerate all nodes
    graph = createGraph(False)
    unvisited = [node.current for node in graph.nodes]
    
    # generate a start point
    start_x = random.choice(X_GRID) + CELLSIZE//2
    start_y = random.choice(Y_GRID) + CELLSIZE//2
    start = vector(start_x, start_y)
    
    # mark it visited
    graph.SetVisited(start)
    unvisited.remove(start)
    frames = [Frame(start, deepcopy(graph))]
    
    print("Generating frames (this will take a while)...")
    with Timer() as t:
        frames = dfs(start, graph, unvisited, frames)
    
    cur = 0  # initialize frame
    c_max = len(frames)
    frame = frames[cur]
    
    print('Playing back at %d fps...' % FPS)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                closeGame()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    closeGame()
        
        sys.stdout.write('\r')
        sys.stdout.write('Current frame: %d/%d' % (cur+1, c_max))
        sys.stdout.flush()
        
        screen.fill(BGCOLOR)
        drawMazeNodes(frame)
        
        pygame.display.update()
        pygame.image.save(screen, './capture/maze_%04d.tga' % (cur+1))
        clock.tick(FPS)
        
        cur += 1  # update frame
        if cur == c_max:
            print('\n')
            return frame
        else:
            frame = frames[cur]
        
# ----- Randomized Prim -----

def prim(graph):
    """Subroutine for generating a maze
    using randomized Prim's algorithm."""
    
    # initialization
    unvisited = [node.current for node in graph.nodes]
    frames = []
    
    # select a starting cell
    start_x = random.choice(X_GRID) + CELLSIZE//2
    start_y = random.choice(Y_GRID) + CELLSIZE//2
    start = vector(start_x, start_y)
    
    # mark it visited
    graph.SetVisited(start)
    unvisited.remove(start)
    frames.append(Frame(start, deepcopy(graph)))
    
    while unvisited != []:
        newCell = random.choice(unvisited)
        neighbors = validNeighbors(newCell)
        random.shuffle(neighbors)  # this is very important
        for neighbor in neighbors:
            if graph.isVisited(neighbor):
                graph.SetConnected(newCell, neighbor)
                graph.SetVisited(newCell)
                unvisited.remove(newCell)
                frames.append(Frame(newCell, deepcopy(graph)))
                break  # do not connect the other visited neighbors
    
    # this is more efficient than using a list comprehension of
    # while unvisited != []:
    #   newCell = random.choice(unvisited)
    #   visitedNeighbors = [N for N in validNeighbors(newCell) if graph.isVisited(N)]
    #   if visitedNeighbors != []:
    #       visited = random.choice(visitedNeighbors)
    #       graph.SetConnected(newCell, visited)
    #       graph.SetVisited(newCell)
    #       unvisited.remove(newCell)
    #       frames.append(Frame(newCell, graph))
    
    return frames

def runPrim():
    """Subroutine for creating a graph
    and running Prim's algorithm."""
    
    # create a graph
    graph = createGraph(False)
    
    print("Generating frames (this will take a while)...")
    with Timer() as t:
        frames = prim(graph)
    
    cur = 0  # initialize frame
    c_max = len(frames)
    frame = frames[cur]
    
    print('Playing back at %d fps...' % FPS)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                closeGame()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    closeGame()
        
        sys.stdout.write('\r')
        sys.stdout.write('Current frame: %d/%d' % (cur+1, c_max))
        sys.stdout.flush()
        
        screen.fill(BGCOLOR)
        drawMazeNodes(frame)
        
        pygame.display.update()
        pygame.image.save(screen, './capture/maze_%04d.tga' % (cur+1))
        clock.tick(FPS)
        
        cur += 1  # update frame
        if cur == c_max:
            print('\n')
            return frame
        else:
            frame = frames[cur]
        
# ----- Recursive Division -----

def divide(graph, X_range, Y_range, frames=[]):
    """Subroutine that divides a graph recursively."""
    
    if len(X_range) == 1 or len(Y_range) == 1:
        # do not return frames here
        # you have to backtrack each subarea
        return None
    
    # select a pivot and a split line
    # that should not be opened
    pivot = vector(random.choice(X_range[1:]), random.choice(Y_range[1:]))
    no_open = random.choice(['north', 'south', 'west', 'east'])
    # no_open = None  # all four split lines must be opened
    
    # obtain current boundaries
    X_lower, Y_lower = X_range[0], Y_range[0]
    X_upper, Y_upper = X_range[-1] + CELLSIZE, Y_range[-1] + CELLSIZE
    
    # create the north split line
    X = pivot.x
    Y_range = range(pivot.y, Y_upper, CELLSIZE)
    Y_skip = None if no_open == 'north' else random.choice(Y_range)
    
    for Y in Y_range:
        if Y != Y_skip:
            cell_W = vector(X - CELLSIZE//2, Y + CELLSIZE//2)
            cell_E = vector(X + CELLSIZE//2, Y + CELLSIZE//2)
            graph.SetApart(cell_W, cell_E)
            frames.append(deepcopy(graph))
    
    # create the south split line
    X = pivot.x
    Y_range = range(Y_lower, pivot.y, CELLSIZE)
    Y_skip = None if no_open == 'south' else random.choice(Y_range)
    
    for Y in Y_range:
        if Y != Y_skip:
            cell_W = vector(X - CELLSIZE//2, Y + CELLSIZE//2)
            cell_E = vector(X + CELLSIZE//2, Y + CELLSIZE//2)
            graph.SetApart(cell_W, cell_E)
            frames.append(deepcopy(graph))
    
    # create the west split line
    Y = pivot.y
    X_range = range(X_lower, pivot.x, CELLSIZE)
    X_skip = None if no_open == 'west' else random.choice(X_range)
    
    for X in X_range:
        if X != X_skip:
            cell_S = vector(X + CELLSIZE//2, Y - CELLSIZE//2)
            cell_N = vector(X + CELLSIZE//2, Y + CELLSIZE//2)
            graph.SetApart(cell_S, cell_N)
            frames.append(deepcopy(graph))
    
    # create the east split line
    Y = pivot.y
    X_range = range(pivot.x, X_upper, CELLSIZE)
    X_skip = None if no_open == 'east' else random.choice(X_range)
    
    for X in X_range:
        if X != X_skip:
            cell_S = vector(X + CELLSIZE//2, Y - CELLSIZE//2)
            cell_N = vector(X + CELLSIZE//2, Y + CELLSIZE//2)
            graph.SetApart(cell_S, cell_N)
            frames.append(deepcopy(graph))
    
    # process each subarea recursively
    X_range = range(X_lower, pivot.x, CELLSIZE)
    Y_range = range(Y_lower, pivot.y, CELLSIZE)
    divide(graph, X_range, Y_range, frames)  # southwest
    
    X_range = range(pivot.x, X_upper, CELLSIZE)
    Y_range = range(Y_lower, pivot.y, CELLSIZE)
    divide(graph, X_range, Y_range, frames)  # southeast
    
    X_range = range(X_lower, pivot.x, CELLSIZE)
    Y_range = range(pivot.y, Y_upper, CELLSIZE)
    divide(graph, X_range, Y_range, frames)  # northwest
    
    X_range = range(pivot.x, X_upper, CELLSIZE)
    Y_range = range(pivot.y, Y_upper, CELLSIZE)
    divide(graph, X_range, Y_range, frames)  # northeast
    
    return frames

def runDivide():
    """Subroutine for creating a graph
    and running recursive division."""
    
    # create a graph
    graph = createGraph(True)
    
    print("Generating frames (this will take a while)...")
    with Timer() as t:
        frames = divide(graph, X_GRID, Y_GRID, [deepcopy(graph)])
    
    cur = 0  # initialize frame
    c_max = len(frames)
    frame = frames[cur]
    
    print('Playing back at %d fps...' % FPS)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                closeGame()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    closeGame()
        
        sys.stdout.write('\r')
        sys.stdout.write('Current frame: %d/%d' % (cur+1, c_max))
        sys.stdout.flush()
        
        screen.fill(BGCOLOR)
        drawMazeWalls(frame)
        
        pygame.display.update()
        pygame.image.save(screen, './capture/maze_%04d.tga' % (cur+1))
        clock.tick(FPS)
        
        cur += 1  # update frame
        if cur == c_max:
            print('\n')
            return frame
        else:
            frame = frames[cur]
        
# ----- Randomized Kruskal -----

def makeWalls(X, Y) -> list:
    """Returns two wall instances if vector(X, Y) is an internal
    intersect, one (or zero) wall instance if it is at the boundary
    of the canvas; however, the result must always be a list."""
    
    # extend northward
    start = vector(X, Y)
    end = vector(X, Y + CELLSIZE)
    wall_N = Wall(start, end)
    
    # extend eastward
    start = vector(X, Y)
    end = vector(X + CELLSIZE, Y)
    wall_E = Wall(start, end)
    
    walls = [wall_N, wall_E]
    if X == 0:  # west boundary
        walls.remove(wall_N)
    if Y == 0:  # south boudanry
        walls.remove(wall_E)
    
    return walls

def getCells(wall):
    """Determines the two adjacent cells
    of a wall as vector instances."""
    
    center_x = (wall.start.x + wall.end.x) // 2
    center_y = (wall.start.y + wall.end.y) // 2
    
    if wall.start.x == wall.end.x:  # vertically oriented
        cell_1 = vector(center_x - CELLSIZE//2, center_y)  # cell west
        cell_2 = vector(center_x + CELLSIZE//2, center_y)  # cell east
    elif wall.start.y == wall.end.y:  # horizontally oriented
        cell_1 = vector(center_x, center_y + CELLSIZE//2)  # cell north
        cell_2 = vector(center_x, center_y - CELLSIZE//2)  # cell south
    
    return cell_1, cell_2

def kruskal(graph):
    """Subroutine for generating a maze
    using randomized Kruskal's algorithm."""
    
    # initialization
    walls = deepcopy(graph.walls)
    random.shuffle(walls)
    
    # this has the same effect as but is more efficient than
    # wall = random.choice(walls) and then walls.remove(wall)
    
    # create a disjoint set
    dset = DisjointSet([{node.current} for node in graph.nodes])
    
    frames = [deepcopy(graph)]
    for wall in walls:
        # find its adjacent cells
        cell_1, cell_2 = getCells(wall)
        
        if dset.AreSeparated(cell_1, cell_2):
            # walls.remove(wall)
            graph.RemoveWall(wall)
            dset.JoinSets(cell_1, cell_2)
            graph.SetConnected(cell_1, cell_2)  # required by maze solvers
            frames.append(deepcopy(graph))
        
        if len(dset) == 1:
            # print("All cells are joint!")
            break
    
    return frames

def runKruskal():
    """Subroutine for creating a graph
    and running Kruskal's algorithm."""
    
    # create a graph with walls
    graph = createGraph(False, True)
    
    print("Generating frames (this will take a while)...")
    with Timer() as t:
        frames = kruskal(graph)
    
    cur = 0  # initialize frame
    c_max = len(frames)
    frame = frames[cur]
    
    print('Playing back at %d fps...' % FPS)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                closeGame()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    closeGame()
        
        sys.stdout.write('\r')
        sys.stdout.write('Current frame: %d/%d' % (cur+1, c_max))
        sys.stdout.flush()
        
        screen.fill(BGCOLOR)
        drawMazeWalls(frame)
        
        pygame.display.update()
        pygame.image.save(screen, './capture/maze_%04d.tga' % (cur+1))
        clock.tick(FPS)
        
        cur += 1  # update frame
        if cur == c_max:
            print('\n')
            return frame
        else:
            frame = frames[cur]
        
# ----- A* Search -----

def manhattan(node_1, node_2):
    """Cost computation for the case that
    only four directions' moves are permitted."""
    
    dist_x = abs(node_2.x - node_1.x)
    dist_y = abs(node_2.y - node_1.y)
    
    return dist_x + dist_y

def astar(maze, start, current, end, visited=[], path=[]):
    """Subroutine for finding the optimum path in a maze
    by computing the cost of each move."""
    
    if current == end:
        return visited, path
    
    # define anonymous functions
    # G = partial(manhattan, node_1=start)
    # H = partial(manhattan, node_2=end)
    
    # enumerate unvisited connected neighbors
    choices = [N for N in maze.GetNeighbors(current) \
        if N not in visited]
    
    # when two nodes have the same H
    # the one with the smaller G comes first
    choices.sort(key=lambda c: manhattan(start, c))
    choices.sort(key=lambda c: manhattan(c, end))
    
    for next_move in choices:
        
        visited.append(next_move)
        path.append(next_move)
        v_final, p_final = \
            astar(maze, start, next_move, end, visited, path)
        
        if p_final is not None:
            return v_final, p_final
        else:
            path.pop()
    
    return visited, None

def convertToFrames(visited, path):
    """Subroutine for converting the output
    into something that can be rendered."""
    
    assert path is not None, "Path is not found!"
    N_visited, N_path = len(visited), len(path)
    paths = []
    
    print("Converting into frames...")
    
    # the first path consists of
    # growing visited cells and no path cell
    for i in range(N_visited):
        paths.append(Path(visited[:i+1], []))
    
    # the second path consists of
    # all visited cells and growing path cells
    for i in range(N_path):
        paths.append(Path(visited, path[:i+1]))
    
    # create a duplicate of the last frame
    # paths.append(Path(visited, path))  # pygame bug
    
    msg_1 = '%d visited cells and ' % N_visited
    msg_2 = '%d in the shortest path.\n' % N_path
    print(msg_1 + msg_2)
    
    return paths

def runAstar(last_frame):
    """Subroutine for running A* Search."""
    # last_frame could be either
    # a frame instance or a maze instance
    
    # select a starting point and an end point
    start = vector(CELLSIZE//2, CELLSIZE//2)
    end = vector(PYGAMEWIDTH - CELLSIZE//2, PYGAMEHEIGHT - CELLSIZE//2)
    
    # initialize parameters
    current = deepcopy(start)
    visited, path = [start], [start]
    
    # unpack the maze
    if isinstance(last_frame, Frame):
        maze = last_frame.maze
        render_type = "drawMazeNodes(last_frame)"
    elif isinstance(last_frame, Maze):
        maze = last_frame
        render_type = "drawMazeWalls(last_frame)"
    
    print("Using A* Search...")
    with Timer() as t:
        visited, path = astar(maze, start, current, end, visited, path)
    
    # convert the solution into frames
    paths = convertToFrames(visited, path)
    
    cur = 0  # initialize frame
    c_max = len(paths)
    path_ = paths[cur]
    
    print('Playing back at %d fps...' % FPS)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                closeGame()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    closeGame()
        
        sys.stdout.write('\r')
        sys.stdout.write('Current frame: %d/%d' % (cur+1, c_max))
        sys.stdout.flush()
        
        screen.fill(BGCOLOR)
        exec(render_type)
        drawPath(path_)
        
        pygame.display.update()
        pygame.image.save(screen, './capture/path_%04d.tga' % (cur+1))
        clock.tick(FPS)
        
        cur += 1  # update frame
        if cur == c_max:
            print('\n')
            return
        else:
            path_ = paths[cur]
        
