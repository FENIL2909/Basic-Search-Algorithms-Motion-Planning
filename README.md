
## Overview

I have implemented **BFS**, **DFS**, **Dijkstra** and **A*** algorithms with Python 3. These are the basic algorithms for discrete planning, which also share a lot of similarities. 

Files included:

- **search.py** is the file where the algorithms are implemented. 
- **main.py** is the scrip that provides helper functions that load the map from csv files and visualize the map and path. 
- **map.csv** is the map file you could modify to create your own map.
- **test_map.csv** restores a test map for doc test purpose only. 

---

For visualization, please run:

`python main.py`

There should be 4 maps shown representing the results of 4 algorithms. The **main.py** loads the map file **map.csv**, which you are free to modify to create your own map.

## More details

- Keep in mind that, the coordinate system used here is **[row, col]**, which could be different from [x, y] in Cartesian coordinates. 
- For Dijkstra and A*, the cost to move each step is set to be 1. The heuristic for each node is its Manhattan distance to the goal.
- When exploring the nearby nodes in the map, I follow this order **"right, down, left, up"**, which means "[0, +1], [+1, 0], [0, -1], [-1, 0]" in coordinates. There is nothing wrong using other exploring orders. It is just that the test result was gotten by algorithms using this order.

