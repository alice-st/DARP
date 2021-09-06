import numpy as np
from Edges import Edge, Graph
import sys


class CalculateTrajectories():
    def __init__(self, r, c, MST):
        self.MAX_NODES = 4*r*c
        self.PathSequence = []
        self.rows = r
        self.cols = c
        self.MSTvector = MST
        self.MSTedges = len(self.MSTvector)
        self.allEdges = set()
        self.nodes = {}
        for node in range(self.MAX_NODES):
            self.nodes[node] = None

    def initializeGraph(self, A, connect4):
        for i in range(2*self.rows):
            for j in range(2*self.cols):
                if A[i, j]:
                    if i > 0 and A[i-1][j]:
                        self.AddToAllEdges(i*2*self.cols+j, (i-1)*2*self.cols+j, 1)
                    if i < 2*self.rows-1 and A[i+1][j]:
                        self.AddToAllEdges(i*2*self.cols+j, (i+1)*2*self.cols+j, 1)
                    if j > 0 and A[i][j-1]:
                        self.AddToAllEdges(i*2*self.cols+j, i*2*self.cols+j-1, 1)
                    if j < 2*self.cols-1 and A[i][j+1]:
                        self.AddToAllEdges(i*2*self.cols+j, i*2*self.cols+j+1, 1)

                    if not connect4:
                        if i > 0 and j > 0 and A[i-1][j-1]:
                            self.AddToAllEdges(i*2*self.cols+j, (i-1)*2*self.cols+j-1, 1)
                        if i < 2*self.rows-1 and j < 2*self.cols-1 and A[i+1][j+1]:
                            self.AddToAllEdges(i*2*self.cols+j, (i+1)*2*self.cols+j+1, 1)
                        if i > 2*self.rows-1 and j > 0 and A[i+1][j-1]:
                            self.AddToAllEdges(i*2*self.cols+j, (i+1)*2*self.cols+j-1, 1)
                        if i > 0 and j < 2*self.cols-1 and A[i-1][j+1]:
                            self.AddToAllEdges(i*2*self.cols+j, (i-1)*2*self.cols+j+1, 1)

    def AddToAllEdges(self, _from: int, to: int, cost):
        self.allEdges.add(Edge(_from, to, cost))

        if (self.nodes[_from]) is None:
            self.nodes[_from] = set()

        self.nodes[_from].add(to)

        if (self.nodes[to]) is None:
            self.nodes[to] = set()

        self.nodes[to].add(_from)

    def RemoveTheAppropriateEdges(self):
        for i in range(self.MSTedges):
            e = self.MSTvector[i]
            maxN = max(e.src, e.dst)
            minN = min(e.src, e.dst)

            if np.absolute(e.src - e.dst) == 1:
                alpha = (4*minN+3) - 2*(maxN % self.cols)
                eToRemove = Edge(alpha, alpha+2*self.cols, 1)
                eToRemoveMirr = Edge(alpha+2*self.cols, alpha, 1)
                eToRemove2 = Edge(alpha+1, alpha+1+2*self.cols, 1)
                eToRemove2Mirr = Edge(alpha+1+2*self.cols, alpha+1, 1)

            else:
                alpha = (4*minN+2*self.cols) - 2*(maxN % self.cols)
                eToRemove = Edge(alpha, alpha+1, 1)
                eToRemoveMirr = Edge(alpha+1, alpha, 1)
                eToRemove2 = Edge(alpha+2*self.cols, alpha+1+2*self.cols, 1)
                eToRemove2Mirr = Edge(alpha+1+2*self.cols, alpha+2*self.cols, 1)

            if eToRemove in self.allEdges:
                self.SafeRemoveEdge(eToRemove)

            if eToRemoveMirr in self.allEdges:
                self.SafeRemoveEdge(eToRemoveMirr)

            if eToRemove2 in self.allEdges:
                self.SafeRemoveEdge(eToRemove2)

            if eToRemove2Mirr in self.allEdges:
                self.SafeRemoveEdge(eToRemove2Mirr)

    def SafeRemoveEdge(self, curEdge):
        try:
            self.allEdges.remove(curEdge)
            # successful removal from priority queue: allEdges
            if curEdge.dst in self.nodes[curEdge.src]:
                self.nodes[curEdge.src].remove(curEdge.dst)
            if curEdge.src in self.nodes[curEdge.dst]:
                self.nodes[curEdge.dst].remove(curEdge.src)

        except KeyError:
            # This is a serious problem
            print("TreeSet should have contained this element!!")
            sys.exit(1)

    def CalculatePathsSequence(self, StartingNode):

        currentNode = StartingNode
        RemovedNodes = set()
        movement = []
        PathSequence = []

        movement.append(2*self.cols)
        movement.append(-1)
        movement.append(-2*self.cols)
        movement.append(1)

        found = False
        prevNode = 0
        for idx in range(4):
            if (currentNode + movement[idx]) in list(self.nodes[currentNode]):
                prevNode = currentNode + movement[idx]
                found = True
                break

        if not found:
            return

        while True:
            if currentNode != StartingNode:
                RemovedNodes.add(currentNode)

            offset = movement.index(prevNode-currentNode)

            prevNode = currentNode

            found = False
            for idx in range(4):
                if (prevNode+movement[(idx+offset) % 4] in self.nodes[prevNode]) and not (prevNode+movement[(idx+offset) % 4] in RemovedNodes):

                    currentNode = prevNode + movement[(idx+offset) % 4]
                    found = True
                    break

            if not found:
                return

            if (prevNode in self.nodes[currentNode]):
                self.nodes[currentNode].remove(prevNode)

            if (currentNode in self.nodes[prevNode]):
                self.nodes[prevNode].remove(currentNode)

            i = int(currentNode/(2*self.cols))
            j = currentNode % (2*self.cols)
            previ = int(prevNode/(2*self.cols))
            prevj = prevNode % (2*self.cols)
            self.PathSequence.append((previ, prevj, i, j))

