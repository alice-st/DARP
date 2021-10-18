from Edges import Edge, Graph
import sys


class Kruskal(object):
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.allEdges = []
        self.MAX_NODES = self.rows*self.cols
        self.nodes = {}
        for node in range(self.MAX_NODES):
            self.nodes[node] = None
        self.mst = []

    def initializeGraph(self, A, connect4, mode):
        cost1 = 1
        cost2 = 1

        for i in range(self.rows):
            for j in range(self.cols):
                if (A[i][j]):
                    if (mode == 0):
                        cost2 = self.rows - i
                    elif mode == 1:
                        cost2 = i+1
                    elif mode == 2:
                        cost1 = self.cols - j
                    elif mode == 3:
                        cost1 = j+1

                    if (i > 0 and A[i-1][j]):
                        self.AddToAllEdges(i*self.cols+j, (i-1)*self.cols+j, cost1)
                    if (i < self.rows-1 and A[i+1][j]):
                        self.AddToAllEdges(i*self.cols+j, (i+1)*self.cols+j, cost1)
                    if (j > 0 and A[i][j-1]):
                        self.AddToAllEdges(i*self.cols+j, i*self.cols+j-1, cost2)
                    if (j < self.cols-1 and A[i][j+1]):
                        self.AddToAllEdges(i*self.cols+j, i*self.cols+j+1, cost2)

                    if not connect4:
                        if (i > 0 and j > 0 and A[i-1][j-1]):
                            AddToAllEdges(i*self.cols+j, (i-1)*self.cols+j-1, 1)
                        if (i < rows-1 and j < self.cols-1 and A[i+1][j+1]):
                            AddToAllEdges(i*self.cols+j, (i+1)*self.cols+j+1, 1)
                        if (i > rows-1 and j > 0 and A[i+1][j-1]):
                            AddToAllEdges(i*self.cols+j, (i+1)*self.cols+j-1, 1)
                        if (i > 0 and j < self.cols-1 and A[i-1][j+1]):
                            AddToAllEdges(i*self.cols+j, (i-1)*self.cols+j+1, 1)

    def AddToAllEdges(self, _from: int, to: int, cost):
        self.allEdges.insert(0, Edge(_from, to, cost))

        if (self.nodes[_from]) is None:
            self.nodes[_from] = set()
            self.nodes[_from].add(_from)

        if (self.nodes[to]) is None:
            self.nodes[to] = set()
            self.nodes[to].add(to)

        return

    def performKruskal(self):
        g1 = Graph(self.nodes, self.allEdges)
        g1.KruskalMST()
        self.mst = g1.mst