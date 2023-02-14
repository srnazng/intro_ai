from Grid import Grid 

def search():
    return

def h():
    return

# a and b are tuples (x,y)
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def forward():
    # search A to T
    return

def backward():
    # search T to A
    return

def f(x, y):
    if g[x][y] >= 0 and  h[x][y] >= 0: 
        return g[x][y] + h[x][y]
    if g[x][y] >= 0:
        # TODO: calculate h value if nonexistent
        return 
    return -1 # no g value

grid = Grid(101)
grid.print_grid()

g = [[-1]*grid.size for _ in range(grid.size)] # store g values
h = [[-1]*grid.size for _ in range(grid.size)] # store heuristics

node_list = [[0]*grid.size for _ in range(grid.size)] # 0 -> uncompleted, 1 -> completed