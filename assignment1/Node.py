import sys

class Node:
    def __init__(self, x, y):
        self.g = sys.maxsize/2
        self.h = -1
        self.blocked = False
        self.x = x
        self.y = y
        self.search = -1 # the last iteration of A* that reached this node
        self.parent = None # the previous node (for backtracking)
    
    def set_g(self, g):
        self.g = g
    
    def set_h(self, h):
        self.h = h 
    
    def set_blocked(self):
        self.blocked = True
