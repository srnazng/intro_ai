import random as rand
from colorama import init
from colorama import Fore, Back, Style

BLOCKED = '#'
UNBLOCKED = ' '
START = 'A'
TARGET = 'T'
dx = [-1, 0, 0, 1] # boundaries of x values
dy = [0, 1, -1, 0] # boundaries of y values

class Grid:
    def print_grid(self):
        print(' ', end='-')
        for col in range(self.size):
            print('-', end='')
        print()
        for row in range(self.size - 1, -1, -1):
            print('|', end=" ")
            for col in range(self.size):
                if self.map[row][col] == START or self.map[row][col] == TARGET:
                    print(Back.GREEN + self.map[row][col], end="")
                else:
                    print(Back.BLACK + self.map[row][col], end="")
            print('|')
        print(' ', end='-')
        for col in range(self.size):
            print('-', end='')
        print()
        print()

    def __init__(self, size):
        self.size = size
        self.map = [[UNBLOCKED]*size for _ in range(size)]
        self.generate_grid()
        self.start = (rand.randrange(self.size), rand.randrange(self.size))

        self.target = (rand.randrange(self.size), rand.randrange(self.size))
        while abs(self.start[0] - self.target[0]) < 1 or abs(self.start[1] - self.target[1]) < 1:
            self.target = (rand.randrange(self.size), rand.randrange(self.size))

        self.map[self.start[0]][self.start[1]] = START
        self.map[self.target[0]][self.target[1]] = TARGET

    def generate_grid(self):
        visited = [[False]*self.size for _ in range(self.size)]
        stack = []
        for i in range(self.size):
            for j in range(self.size):
                if not visited[i][j]:
                    self.dfs((i, j), visited, stack)

    def dfs(self, coord, visited, stack):
        stack.append(coord)

        while stack:
            coord = stack.pop()
            if visited[coord[0]][coord[1]]:
                continue
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
                    stack.append(neighbor)
                else:
                    self.map[neighbor[0]][neighbor[1]] = BLOCKED

init()