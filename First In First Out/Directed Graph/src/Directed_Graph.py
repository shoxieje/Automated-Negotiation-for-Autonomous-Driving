
# idea from James -> https://github.com/joeyajames/Python/blob/master/graph_adjacency-list.py

class Vertex:
    def __init__(self, name):
        self.__name = name
        self.neighbors = list()

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, value):
        self.__name = value

    def add_neighbors(self, v):
        if v not in self.neighbors:
            self.neighbors.append(v)
            self.neighbors.sort()


class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex, *argv):
        argv = argv + (vertex,)

        # add the vertex to dictionary
        for v in argv:
            if isinstance(v, Vertex) and v.name not in self.vertices:
                self.vertices[v.name] = v


    def add_edges(self, u, v):
        if u in self.vertices and v in self.vertices and u != v:
            self.vertices[u].add_neighbors(v)


    def print_graph(self):
        a = []
        for key in self.vertices.keys():
            a.append(str(key) + ' ' + str(self.vertices[key].neighbors))

        return ''.join(a)
