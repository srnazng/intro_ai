from GridWorld import GridWorld
from Heap import MinHeap
from Node import Node
import sys

dx = [-1, 0, 0, 1] # boundaries of x values
dy = [0, 1, -1, 0] # boundaries of y values

class Run:
    def __init__(self, size):
        self.size = size
        self.gridworld = GridWorld(size)
        self.gridworld.print_grid()
        self.node_grid = [[Node(i, j) for j in range(size)] for i in range(size)] # 2d Array of Nodes with currently known values
        self.start = self.node_grid[self.gridworld.start[0]][self.gridworld.start[1]]
        self.target = self.node_grid[self.gridworld.target[0]][self.gridworld.target[1]]
        self.print_nodes()

        # initialize h values for each node
        for row in self.node_grid:
            for node in row:
                node.h = self.h(node)

    def print_nodes(self):
        for r in self.node_grid:
            for elem in r:
                if self.f(elem) < sys.maxsize/2:
                    print('(' + str(elem.x) + ',' + str(elem.y) + '): ' + str(self.f(elem)), end=' ') #(row, column)
                else:
                    print('(' + str(elem.x) + ',' + str(elem.y) + '): ' + str(-1), end=' ') #(row, column)
                if elem.blocked:
                    print('# |', end=' ')
                else:
                    print('_ |', end = ' ')
            print()

    def compare(self, n1, n2):
        return self.f(n1) - self.f(n2)

    def search(self):
        return

    def h(self, node):
        return self.manhattan(node, self.target)

    def manhattan(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def repeated_forward(self):
        final_path = []
        counter = 0
        current_start = self.start
        while True:
            # if current start is target then return final path
            if current_start == self.target:
                return final_path
            
            # run A* on current map
            path = self.forward(current_start, counter)

            # if forward returns None --> there is no path and return None
            if self.forward() is None:
                return None 

            # update counter
            counter += 1

            # while not at dead end
                # check if next step in path is valid by expanding current start
                # if valid update map (and update search value to each node it encounters)
            for i in (len(path) - 1):
                current_start = path[0]
                

            # if not valid step --> rerun forward() with same start

            # else --> update final_path and update current start
            
    # Single instance of A* on current map
    def forward(self, current_start, counter):
        # Search A to T
        flag = False # If G has been found

        # Create new open list (min heap)
        open_list = MinHeap(4 * self.size * self.size, self.compare)

        # Update g values of start and target
        self.target.g = sys.maxsize
        self.target.search = counter
        current_start.g = 0
        current_start.search = counter
        
        # Insert starting node of A* as the first node
        open_list.insert(current_start)

        
        # find shortest path
        while self.target.g > (self.open_list.returnMin().g + self.open_list.returnMin().h):
            # get node with smallest f, this node gets EXPANDED.
            min_node = self.MinHeap.remove()

            # target found
            if min_node == self.target:
                flag = True
                break
            
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
                    self.MinHeap.insert(adj_node)

        # no path to the goal was found, so return with nothing
        if not flag:
            return None

        # trace backwards from target to start
        path = []
        curr_node = self.target
        while curr_node is not current_start:
            path.append(curr_node)
            curr_node = curr_node.parent
            
        return path[::-1]

    def repeated_backward(self):
        temp = self.target
        self.target = self.start
        self.start = temp
        return repeated_forward()

    def f(self, node):
        if node.h < 0:
            node.h = self.h(node)
        if node.g >= 0:
            return node.g + node.h
        return -1

Run(10)