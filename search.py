# Basic searching algorithms

# Class for each node in the grid
import operator

import numpy as np


class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node


# check if the grid cell under exploration is inside the grid
def isValid(row,col,ROW,COL):
    if((row>=0) and (row<ROW) and (col>=0) and (col<COL)):
        return True
    else:
        return False

# order of exploration    ### right, down, left, up ###
exploreRows = [0,1,0,-1]
exploreCols = [1,0,-1,0]

def searchPath(grid, start, goal, type):
    # Calculating the Grid Size
    ROW = len(grid)
    COL = len(grid[0])
    # Initializing the Start and the Goal Node
    Start = Node(start[0], start[1], False, 0)
    Start.g = 0
    Goal = Node(goal[0], goal[1], False, 0)
    Goal.g = 10e9

    def manhattanDistance(row,col):
        return abs(Goal.row - row) + abs(Goal.col - col)

    # Initializing a List to keep track of the visited nodes
    visited = []
    for i in range(0, ROW):
        temp = []
        for j in range(0, COL):
                temp.append(False)
        visited.append(temp)

    distance = []
    for i in range(0, ROW):
        temp = []
        for j in range(0, COL):
                temp.append(float('inf'))
        distance.append(temp)

    # Initializations
    queue = []
    found = False
    path = []
    steps = 0

    # Handling Boundary Cases
    errorFlag = False
    if ((not isValid(Start.row, Start.col, ROW, COL)) or (
            not isValid(Goal.row, Goal.col, ROW, COL)) or Start.is_obs or Goal.is_obs):
        errorFlag = True
    else:
        visited[Start.row][Start.col] = True  # Toggling the start to be a visited cell

        distance[Start.row][Start.col] = 0  # Initializing the start node with zero distance
        Start.g = distance[Start.row][Start.col]
        Start.h = manhattanDistance(Start.row, Start.col)
        Start.cost = Start.g + Start.h
        if type == "dijkstra":
            queue.append([Start.g, Start])  # appending start node to the queue
        elif type == "astar":
            queue.append([Start.cost, Start])  # appending start node to the queue
        else:
            queue.append(Start)  # appending start node to the queue

    while (len(queue) != 0 and not (errorFlag)):
        if type == "BFS":
            currNode = queue[0]
        elif type == "DFS":
            currNode = queue[-1]
            path.append([currNode.row, currNode.col])
        else:
            queue.sort(key=operator.itemgetter(0))
            currNode = queue[0][1]


        steps = steps + 1  # incrementing when we explore the node
        if (currNode.row == Goal.row and currNode.col == Goal.col):

            if type != "DFS":
                # Printing out Path
                tempNode = currNode
                path.insert(0, [tempNode.row, tempNode.col])
                while tempNode.parent != None:
                    path.insert(0, [tempNode.parent.row, tempNode.parent.col])
                    tempNode = tempNode.parent

            # Terminating the loop
            found = True
            break

        if type == "DFS":
            queue.pop()
        else:
            queue.pop(0)

        # Visiting the adjcent nodes and storing them in the queue
        for i in range(0, len(exploreRows)):
            if type == "DFS":
                row = currNode.row + exploreRows[3-i]
                col = currNode.col + exploreCols[3-i]
            else:
                row = currNode.row + exploreRows[i]
                col = currNode.col + exploreCols[i]


            if ((isValid(row, col, ROW, COL)) and (grid[row][col] == 0)):
                if(((type == "BFS" or type == "DFS") and (not visited[row][col])) or ((type == "dijkstra" or type == "astar") and (distance[row][col] > distance[currNode.row][currNode.col] + 1))) :
                    visited[row][col] = True
                    distance[row][col] = distance[currNode.row][currNode.col] + 1  # adding 1 as all the cost between nodes 1
                    adjNode = Node(row, col, False, 0)
                    adjNode.parent = currNode
                    adjNode.g = distance[row][col]
                    adjNode.h = manhattanDistance(row, col)
                    adjNode.cost = adjNode.g + adjNode.h
                    if type == "dijkstra":
                        queue.append([adjNode.g, adjNode])
                    elif type == "astar":
                        queue.append([adjNode.cost, adjNode])
                    else:
                        queue.append(adjNode)
    # print(type)
    # print(np.asmatrix(distance))
    # print(np.asmatrix(visited))
    return found, path, steps



def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    found = False
    path = []
    steps = 0

    found, path, steps = searchPath(grid, start, goal, "BFS")
    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    # Initializations
    found = False
    path = []
    steps = 0

    found, path, steps = searchPath(grid, start, goal, "DFS")

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''

    ### YOUR CODE HERE ###
    found, path, steps = searchPath(grid, start, goal, "dijkstra")
    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''

    ### YOUR CODE HERE ###
    found, path, steps = searchPath(grid, start, goal, "astar")
    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
