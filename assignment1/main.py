from GridWorld import GridWorld
from Heap import MinHeap
from Node import Node
import sys

dx = [-1, 0, 0, 1] # boundaries of x values
dy = [0, 1, -1, 0] # boundaries of y values

class Run:
    def __init__(self, size, forward=True, adaptive=False):
        self.adaptive = adaptive
        self.forward = forward
        self.size = size
        self.gridworld = GridWorld(size)
        self.gridworld.print_grid()
        self.node_grid = [[Node(i, j) for j in range(size)] for i in range(size)] # 2d Array of Nodes with currently known values
        self.start = self.node_grid[self.gridworld.start[0]][self.gridworld.start[1]]
        self.target = self.node_grid[self.gridworld.target[0]][self.gridworld.target[1]]

        # initialize h values for each node
        for row in self.node_grid:
            for node in row:
                node.h = self.h(node)

        self.a_star()

    def a_star(self):
        path = self.repeated_a_star(self.forward)
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
                print('(' + str(node.x) + ', ' + str(node.y) + ')', end=' ')
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

    def compare(self, n1, n2):
        #in favor of larger g values
        return (self.size**2*self.f(n1) - n1.g)-(self.size**2*self.f(n2)-n2.g)

    def compare_smaller(self, n1, n2):
        #in favor of smaller g values
        return (self.size**2*self.f(n1) - n1.g)-(self.size**2*self.f(n2)-n2.g)

    def h(self, node):
        if self.adaptive:
            return self.adaptive_h(node)
        return self.manhattan(node, self.target)

    def manhattan(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def adaptive_h(self, node):
        return self.target.g - node.g

    def repeated_a_star(self, forward):
        final_path = []
        
        counter = 0
        current_start = self.start
        final_path.append(current_start)

        while True:
            # if current start is target then return final path
            if current_start == self.target:
                return final_path
            
            # run A* on current map
            if forward:
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
                    adj_node.parent = min_node
                    open_list.insert(adj_node)

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
                    adj_node.parent = min_node
                    open_list.insert(adj_node)

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
            
        return path[::-1]

    def f(self, node):
        if node.g >= 0:
            return node.g + node.h
        return -1

Run(101, False, True)