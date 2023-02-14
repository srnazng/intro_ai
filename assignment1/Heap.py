import sys

class MinHeap:
    def __init__(self, maxsize, compare):
        self.maxsize = maxsize
        self.size = 0
        self.Heap = [0]*(self.maxsize + 1)
        self.Heap[0] = -1 * sys.maxsize
        self.FRONT = 1

    # Function: return position of parent for node at pos
    def parent(self, pos):
        return pos//2

    # Function: return position of left child of node at pos
    def leftChild(self, pos):
        return 2 * pos

    # Function: return position of right child of node at pos
    def rightChild(self, pos):
        return (2 * pos) + 1

    # Function: returns true if passed node is leaf node
    def isLeaf(self, pos):
        return pos*2 > self.size

    # Function: swap two nodes
    def swap(self, fpos, spos):
        self.Heap[fpos], self.Heap[spos] = self.Heap[spos], self.Heap[fpos]

    # Function: heapify node at pos
    def minHeapify(self, pos):
        # If node is non-leaf node + > than any of children
        if not self.isLeaf(pos):
            if compare(self.Heap[pos], self.Heap[self.leftChild(pos)]) > 0 or compare(self.Heap[pos], self.Heap[self.rightChild(pos)]) > 0: #if pos > left or right child
                # Swap with left child and heapify left child
                if compare(self.Heap[self.leftChild(pos)], self.Heap[self.rightChild(pos)]) < 0: #left < right
                    self.swap(pos, self.leftChild(pos))
                    self.minHeapify(self.leftChild(pos))
                # Swap with right child and heapify right child
                else:
                    self.swap(pos, self.rightChild(pos))
                    self.minHeapify(self.rightChild(pos))

    # Function: insert node into heap
    def insert(self, element):
        if self.size >= self.maxsize :
            return
        self.size += 1
        self.Heap[self.size] = element
        current = self.size
        while compare(self.Heap[current], self.Heap[self.parent(current)]) < 0: #if current < parent of current 
            self.swap(current, self.parent(current))
            current = self.parent(current)

    # Function: print heap
    def Print(self):
        for i in range(1, (self.size//2)+1):
            print(" PARENT : " + str(self.Heap[i]) + " LEFT CHILD : " + str(self.Heap[2 * i]) + " RIGHT CHILD : " +
                str(self.Heap[2 * i + 1]))

    # Function: build min heap using minHeapify
    def minHeap(self):
        for pos in range(self.size//2, 0, -1):
            self.minHeapify(pos)

    # Function: remove minimum element
    def remove(self):
        self.Heap[self.FRONT] = self.Heap[self.size]
        self.size-= 1
        self.minHeapify(self.FRONT)

    # Function: return minimum element
    def returnMin(self):
        minim=self.Heap[self.FRONT]
        return minim