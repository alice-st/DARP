class Edge(object):
    def __init__(self, _from, to, weight):
        self.src = _from
        self.dst = to
        self.weight = weight

    def __eq__(self, other):
        return (self.src == other.src and self.dst == other.dst and self.weight == other. weight)

    def __hash__(self):
        return hash((self.src, self.dst, self.weight))


class Graph:

    def __init__(self, nodes, arg_edgelist):
        self.nodes = nodes
        self.num_nodes = len(self.nodes)
        self.edgelist = arg_edgelist
        self.parent = []
        self.rank = []
        # mst stores edges of the minimum spanning tree
        self.mst = []

    def FindParent(self, node):
        # With path-compression.

        if node != self.parent[node]:
            self.parent[node] = self.FindParent(self.parent[node])
        return self.parent[node]

    def KruskalMST(self):

        # Sort objects of an Edge class based on attribute (weight)
        self.edgelist.sort(key=lambda Edge: Edge.weight)

        self.parent = [None] * self.num_nodes
        self.rank = [None] * self.num_nodes

        for n in self.nodes:
            self.parent[n] = n  # Every node is the parent of itself at the beginning
            self.rank[n] = 0   # Rank of every node is 0 at the beginning

        for edge in self.edgelist:
            root1 = self.FindParent(edge.src)
            root2 = self.FindParent(edge.dst)

            # Parents of the source and destination nodes are not in the same subset
            # Add the edge to the spanning tree
            if root1 != root2:
                self.mst.append(edge)
                if self.rank[root1] < self.rank[root2]:
                    self.parent[root1] = root2
                    self.rank[root2] += 1
                else:
                    self.parent[root2] = root1
                    self.rank[root1] += 1

        cost = 0
        for edge in self.mst:
            cost += edge.weight
