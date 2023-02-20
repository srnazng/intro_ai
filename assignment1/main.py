from GridWorld import GridWorld
from Heap import MinHeap
from Node import Node
import sys
import time
import copy

dx = [-1, 0, 0, 1] # boundaries of x values
dy = [0, 1, -1, 0] # boundaries of y values

class Run:
    def __init__(self, size, forward=True, adaptive=False, favor_larger_g=True, print=True):
        self.adaptive = adaptive
        self.is_forward = forward
        self.size = size
        self.gridworld = GridWorld(size)
        self.print = print
        if print:
            self.gridworld.print_grid()
        self.node_grid = [[Node(i, j) for j in range(size)] for i in range(size)] # 2d Array of Nodes with currently known values
        self.start = self.node_grid[self.gridworld.start[0]][self.gridworld.start[1]]
        self.target = self.node_grid[self.gridworld.target[0]][self.gridworld.target[1]]
        self.expanded_count = 0

        if favor_larger_g:
            self.compare = self.compare_favor_larger_g
        else:
            self.compare = self.compare_favor_smaller_g

        self.calculate_h()

        self.a_star()

    def calculate_h(self):
        # initialize h values for each node
        for row in self.node_grid:
            for node in row:
                node.h = self.h(node)

    def a_star(self):
        path = self.repeated_a_star()
        if self.print:
            if path is None:
                print("I cannot reach the target")
            else:
                for node in path[0:-1]:
                    if node is not self.start:
                        self.gridworld.map[node.x][node.y] = '.'
                self.gridworld.print_grid()

                print("PATH")
                for node in path:
                    self.gridworld.map[node.x][node.y] = '.'
                    print('(' + str(node.x) + ',' + str(node.y) + ')', end=' ')
            print()

    def print_nodes(self):
        for r in self.node_grid:
            for elem in r:
                if self.f(elem) < sys.maxsize/2:
                    print('(' + str(elem.x) + ',' + str(elem.y) + '): ' + str(self.f(elem)) + '=' + str(elem.g) + '+' + str(elem.h), end=' ') #(row, column)
                else:
                    print('(' + str(elem.x) + ',' + str(elem.y) + '): ' + str(-1), end=' ') #(row, column)
                if elem.blocked:
                    print('# |', end=' ')
                else:
                    print('_ |', end = ' ')
            print()

    def compare_favor_larger_g(self, n1, n2):
        #in favor of larger g values
        return (self.size**2 * self.f(n1) - n1.g) - (self.size**2*self.f(n2) - n2.g)

    def compare_favor_smaller_g(self, n1, n2):
        #in favor of smaller g values
        return (self.size**2 * self.f(n1) - n1.g) - (self.size**2*self.f(n2) - n2.g)

    def h(self, node):
        if self.adaptive:
            return self.adaptive_h(node)
        return self.manhattan(node, self.target)

    def manhattan(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def adaptive_h(self, node):
        return self.target.g - node.g

    def repeated_a_star(self):
        final_path = []
        
        counter = 0
        current_start = self.start
        final_path.append(current_start)
        self.expanded_count = 0

        while True:
            # if current start is target then return final path
            if current_start == self.target:
                return final_path
            
            # run A* on current map
            if self.is_forward:
                path = self.forward(current_start, counter)
            else:
                path = self.backward(current_start, counter)

            # if forward returns None --> there is no path and return None
            if path is None:
                return None 

            # for each node in path 
            for i in range(len(path) - 1):    
                # print(f"Current start: {current_start.x}, {current_start.y}")
                # expand current start
                for j in range(4):
                    x = current_start.x + dx[j]
                    y = current_start.y + dy[j]

                    if x < 0 or y < 0 or x >= self.size or y >= self.size:
                        continue
                    if self.gridworld.map[x][y] == '#':
                        # if blocked update map
                        self.node_grid[x][y].blocked = True

                # check if reach dead end update final_path if not dead end
                if path[i + 1].blocked:
                    break
                else:
                    final_path.append(path[i + 1])
                    current_start = path[i + 1]

            # update counter
            counter += 1

        return final_path

    # Single instance of A* on current map
    def backward(self, current_start, counter):
        # Search A to T
        flag = False # If G has been found

        # Create new open list (min heap)
        open_list = MinHeap(4 * self.size * self.size, self.compare)

        # Update g values of start and target
        self.target.g = 0
        self.target.search = counter
        current_start.g = sys.maxsize/2
        current_start.search = counter
        
        # Insert starting node of A* as the first node
        open_list.insert(self.target)
        
        # find shortest path
        while open_list.size > 0 and current_start.g > (open_list.returnMin().g + open_list.returnMin().h):
            # get node with smallest f, this node gets EXPANDED.
            min_node = open_list.returnMin()
            open_list.remove()
            if (min_node.g > self.node_grid[min_node.x][min_node.y].g):
                continue

            self.expanded_count += 1

            # print(f"Popping: {min_node.x}, {min_node.y}. Target: {self.target.x}, {self.target.y}")
            
            # FIND SHORTEST PATH USING CURRENT KNOWLEDGE
            # Loop through all the neighbors of the node. 
            # If the neighbor node isn't blocked with current knowledge, check to see if the path cost (g) through the expanded node
            # is less than the current path cost in the neighbor node. If it is, update the path cost of the node, and 
            # insert it into the heap
            for i in range(4):
                x = min_node.x + dx[i]
                y = min_node.y + dy[i]

                # ensure valid unblocked node
                if x < 0 or y < 0 or x >= self.size or y >= self.size or self.node_grid[x][y].blocked:
                    continue
                adj_node = self.node_grid[x][y]

                # update search number of adj_node
                if adj_node.search < counter:
                    adj_node.g = sys.maxsize/2
                    adj_node.search = counter
                
                # update cost of adj_node and insert into open list
                if min_node.g + 1 < adj_node.g:
                    adj_node.g = min_node.g + 1
                    adj_node.parent = self.node_grid[min_node.x][min_node.y]
                    open_list.insert(copy.deepcopy(adj_node))

        # no path to the goal was found if open list is exhausted, so return with nothing
        if open_list.size == 0:
            return None

        # trace backwards from target to start
        path = []
        curr_node = current_start
        while not curr_node == self.target:
            path.append(curr_node)
            curr_node = curr_node.parent
        path.append(self.target)

        if self.adaptive:
            self.calculate_h()
    
        return path
            
    # Single instance of A* on current map
    def forward(self, current_start, counter):
        # Search A to T
        flag = False # If G has been found

        # Create new open list (min heap)
        open_list = MinHeap(4 * self.size * self.size, self.compare)

        # Update g values of start and target
        self.target.g = sys.maxsize/2
        self.target.search = counter
        current_start.g = 0
        current_start.search = counter
        
        # Insert starting node of A* as the first node
        open_list.insert(current_start)
        
        # find shortest path
        while open_list.size > 0 and self.target.g > (open_list.returnMin().g + open_list.returnMin().h):
            # get node with smallest f, this node gets EXPANDED.
            min_node = open_list.returnMin()
            open_list.remove()
            if (min_node.g > self.node_grid[min_node.x][min_node.y].g):
                continue

            self.expanded_count += 1

            # print(f"Popping: {min_node.x}, {min_node.y}. Target: {self.target.x}, {self.target.y}")
            
            # FIND SHORTEST PATH USING CURRENT KNOWLEDGE
            # Loop through all the neighbors of the node. 
            # If the neighbor node isn't blocked with current knowledge, check to see if the path cost (g) through the expanded node
            # is less than the current path cost in the neighbor node. If it is, update the path cost of the node, and 
            # insert it into the heap
            for i in range(4):
                x = min_node.x + dx[i]
                y = min_node.y + dy[i]

                # ensure valid unblocked node
                if x < 0 or y < 0 or x >= self.size or y >= self.size or self.node_grid[x][y].blocked:
                    continue
                adj_node = self.node_grid[x][y]

                # update search number of adj_node
                if adj_node.search < counter:
                    adj_node.g = sys.maxsize/2
                    adj_node.search = counter
                
                # update cost of adj_node and insert into open list
                if min_node.g + 1 < adj_node.g:
                    adj_node.g = min_node.g + 1
                    adj_node.parent = self.node_grid[min_node.x][min_node.y]
                    open_list.insert(copy.deepcopy(adj_node))

        # no path to the goal was found if open list is exhausted, so return with nothing
        if open_list.size == 0:
            return None

        # trace backwards from target to start
        path = []
        curr_node = self.target
        while not curr_node == current_start:
            path.append(curr_node)
            curr_node = curr_node.parent
        path.append(current_start)

        if self.adaptive:
            self.calculate_h()
            
        return path[::-1]

    def f(self, node):
        if node.g >= 0:
            return node.g + node.h
        return -1

NUM_TRIALS = 10
MAP_WIDTH = 50 # default 101

# DEFAULT
Run(MAP_WIDTH, print=False)

# TEST REPEATED FORWARD A*

total_time_1 = 0.0
total_expanded_1 = 0
for i in range(NUM_TRIALS):
    print(f"Trial {i}")
    start = time.time()
    run_instance = Run(MAP_WIDTH, True, False, True, False) # forward=True, adaptive=False, favor_larger_g=True, print=False
    total_expanded_1 += run_instance.expanded_count
    end = time.time()
    total_time_1 = total_time_1 + (end - start)
avg_time_1 = total_time_1 / NUM_TRIALS
avg_expanded_1 = total_expanded_1 / NUM_TRIALS

# TEST REPEATED BACKWARD A*
# total_time_2 = 0.0
# total_expanded_2 = 0
# for i in range(NUM_TRIALS):
#     print(f"Trial {i}")
#     start = time.time()
#     run_instance = Run(MAP_WIDTH, False, False, True, False) # forward=False, adaptive=False, favor_larger_g=True, print=False
#     total_expanded_2 += run_instance.expanded_count
#     end = time.time()
#     total_time_2 = total_time_2 + (end - start)
# avg_time_2 = total_time_2 / NUM_TRIALS
# avg_expanded_2 = total_expanded_2 / NUM_TRIALS 

# TEST ADAPTIVE A*
total_time_3 = 0.0
total_expanded_3 = 0
for i in range(NUM_TRIALS):
    print(f"Trial {i}")
    start = time.time()
    run_instance = Run(MAP_WIDTH, True, True, True, False) # forward=True, adaptive=True, favor_larger_g=True, print=False
    total_expanded_3 += run_instance.expanded_count
    end = time.time()
    total_time_3 = total_time_3 + (end - start)
avg_time_3 = total_time_3 / NUM_TRIALS
avg_expanded_3 = total_expanded_3 / NUM_TRIALS

# # TEST REPEATED FORWARD A* WITH TIE BREAKING FAVORING SMALLER G
# total_time_4 = 0.0
# for i in range(NUM_TRIALS):
#     start = time.time()
#     Run(MAP_WIDTH, True, False, False, False) # forward=True, adaptive=True, favor_larger_g=False, print=False
#     end = time.time()
#     total_time_4 = total_time_4 + (end - start)
# avg_time_4 = total_time_4 / NUM_TRIALS

# PRINT RESULTS
print("TEST REPEATED FORWARD A*:")
print(f" Average Runtime: {avg_time_1}, Average expanded nodes: {avg_expanded_1}")
# print("TEST REPEATED BACKWARD A*:")
# print(f" Average Runtime: {avg_time_2}, Average expanded nodes: {avg_expanded_2}")
print("TEST ADAPTIVE REPEATECD FORWARD A*:")
print(f" Average Runtime: {avg_time_3}, Average expanded nodes: {avg_expanded_3}")
# print("TEST REPEATED FORWARD A* FAVORING SMALLER G:")
# print(" Average Runtime:", avg_time_4)