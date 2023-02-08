import random as rand

BLOCKED = '#'
UNBLOCKED = ' '
dx = [-1, 0, 0, 1] # boundaries of x values
dy = [0, 1, -1, 0] # boundaries of y values

class Grid:
    def print_grid(self):
        print(' ', end='-')
        for col in range(self.size):
            print('-', end='-')
        print()
        for row in range(self.size - 1, -1, -1):
            print('|', end=" ")
            for col in range(self.size):
                print(self.map[row][col], end=" ")
            print('|')
        print(' ', end='-')
        for col in range(self.size):
            print('-', end='-')
        print()
        print()

    def __init__(self, size):
        self.size = size
        self.map = [[UNBLOCKED]*size for _ in range(size)]
        self.generate_grid()
        self.start = (rand.randrange(10), rand.randrange(10))

        self.target = (rand.randrange(10), rand.randrange(10))
        while abs(self.start[0] - self.target[0]) < self.size/2 or abs(self.start[1] - self.target[1]) < self.size/2:
            self.target = (rand.randrange(10), rand.randrange(10))

        self.map[self.start[0]][self.start[1]] = 'A'
        self.map[self.target[0]][self.target[1]] = 'T'

    def generate_grid(self):
        visited = [[False]*self.size for _ in range(self.size)]
        
        for i in range(self.size):
            for j in range(self.size):
                if not visited[i][j]:
                    self.dfs((i, j), visited)

    def dfs(self, coord, visited):
        if visited[coord[0]][coord[1]]:
            return

        visited[coord[0]][coord[1]] = True

        unvisited_neighbors = []
        for i in range(4):
            new_coord = (coord[0] + dx[i], coord[1] + dy[i])
            if new_coord[0] >= 0 and new_coord[0] < self.size and new_coord[1] >= 0 and new_coord[1] < self.size:
                if not visited[new_coord[0]][new_coord[1]]:
                    unvisited_neighbors.append(new_coord)

        rand.shuffle(unvisited_neighbors)
        for neighbor in unvisited_neighbors:
            if (rand.random() < 0.7):
                self.dfs(neighbor, visited)
            else:
                self.map[neighbor[0]][neighbor[1]] = BLOCKED
