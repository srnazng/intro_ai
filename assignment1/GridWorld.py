import random as rand
from colorama import init
from colorama import Fore, Back, Style

BLOCKED = '#'
UNBLOCKED = ' '
PATH = '.'
START = 'A'
TARGET = 'T'
dx = [-1, 0, 0, 1] # boundaries of x values
dy = [0, 1, -1, 0] # boundaries of y values

class GridWorld:
    def print_grid(self):
        print(Back.WHITE + ' ', end=' ')
        for col in range(self.size):
            print(Back.WHITE + ' ', end='')
        print(Back.WHITE + ' ', end=' ')
        print(Style.RESET_ALL)
        for row in range(self.size):
            print(Back.WHITE + ' ', end=" ")
            for col in range(self.size):
                if self.map[row][col] == START:
                    print(Back.GREEN + self.map[row][col], end="")
                elif self.map[row][col] == TARGET:
                    print(Back.GREEN + self.map[row][col], end="")
                elif self.map[row][col] == BLOCKED:
                    print(Back.RED + " ", end="")
                elif self.map[row][col] == PATH:
                    print(Back.GREEN + " ", end="")
                else:
                    print(Back.BLACK + self.map[row][col], end="")
            print(Back.WHITE + ' ', end=' ')
            print(Style.RESET_ALL)
        print(Back.WHITE + ' ', end=' ')
        for col in range(self.size):
            print(Back.WHITE + ' ', end='')
        print(Back.WHITE + ' ', end=' ')
        print(Style.RESET_ALL)
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

        blocked_count = 0
        for i in range(self.size):
            for j in range(self.size):
                blocked_count += (self.map[i][j] == '#')

        # print(f"Fraction blocked: {blocked_count / (self.size * self.size)}")

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

            neighbors = []
            for i in range(4):
                new_coord = (coord[0] + dx[i], coord[1] + dy[i])
                if new_coord[0] >= 0 and new_coord[0] < self.size and new_coord[1] >= 0 and new_coord[1] < self.size:
                    neighbors.append(new_coord)

            rand.shuffle(neighbors)
            for neighbor in neighbors:
                if not visited[neighbor[0]][neighbor[1]]:
                    if (rand.random() < 0.7):
                        stack.append(neighbor)
                    else:
                        self.map[neighbor[0]][neighbor[1]] = BLOCKED
                        visited[neighbor[0]][neighbor[1]] = True

init()