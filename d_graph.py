# Course: CS261 - Data Structures
# Author: Hye Yeon Park
# Assignment: Graph Implementation - Directed Graph
# Description: Implement the DirectedGraph class, which is designed to support the following type of graph:
# directed, weighted (positive edge weights only), no duplicate edges, no loops. Cycles are allowed.
# This implementation will include the following methods:
#           add_vertex(), add_edge()
#           remove_edge(), get_vertices(), get_edges()
#           is_valid_path(), dfs(), bfs()
#           has_cycle(), dijkstra()


import heapq
from collections import deque


class DirectedGraph:
    """
    Class to implement directed weighted graph
    - duplicate edges not allowed
    - loops not allowed
    - only positive edge weights
    - vertex names are integers
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency matrix
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.v_count = 0
        self.adj_matrix = []

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            v_count = 0
            for u, v, _ in start_edges:
                v_count = max(v_count, u, v)
            for _ in range(v_count + 1):
                self.add_vertex()
            for u, v, weight in start_edges:
                self.add_edge(u, v, weight)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        if self.v_count == 0:
            return 'EMPTY GRAPH\n'
        out = '   |'
        out += ' '.join(['{:2}'.format(i) for i in range(self.v_count)]) + '\n'
        out += '-' * (self.v_count * 3 + 3) + '\n'
        for i in range(self.v_count):
            row = self.adj_matrix[i]
            out += '{:2} |'.format(i)
            out += ' '.join(['{:2}'.format(w) for w in row]) + '\n'
        out = f"GRAPH ({self.v_count} vertices):\n{out}"
        return out

    # ------------------------------------------------------------------ #

    def add_vertex(self) -> int:
        """
        Adds a new vertex to the graph
        Returns a single integer - the number of vertices in the graph
        """
        # Increase v_count by 1
        self.v_count += 1

        # add new row for new vertex
        self.adj_matrix.append([0]*(self.v_count))

        # add a column to the existing rows for added index
        for i in range(self.v_count - 1):
            self.adj_matrix[i].append(0)

        return self.v_count

    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        Adds a new edge to the graph
        """
        # if either vertex indices do not exist in the graph
        if src >= self.v_count or src < 0 or dst >= self.v_count or dst < 0:
            return
        # if the weight is not a positive integer, or if src and dst refer to the same vertex
        if weight <= 0 or src == dst:
            return
        self.adj_matrix[src][dst] = weight

    def remove_edge(self, src: int, dst: int) -> None:
        """
        Removes an edge between two vertices with provided indices
        """
        # If either vertex indices do not exist in the graph
        if src >= self.v_count or src < 0 or dst >= self.v_count or dst < 0:
            return
        # if there is no edge between them
        if self.adj_matrix[src][dst] == 0:
            return
        self.adj_matrix[src][dst] = 0

    def get_vertices(self) -> []:
        """
        Returns a list of vertices of the graph
        """
        result = list(range(0, self.v_count))
        return result

    def get_edges(self) -> []:
        """
        Returns a list of edges in the graph.
        Each edge is returned as a tuple of (src, dst, weight)
        """
        result = []
        for src in range(self.v_count):
            for dst in range(self.v_count):
                if self.adj_matrix[src][dst]:
                    weight = self.adj_matrix[src][dst]
                    result.append((src, dst, weight))
        return result

    def is_valid_path(self, path: []) -> bool:
        """
        Takes a list of vertex indices and returns True if the sequence of vertices
        represents a valid path in the graph
        """
        # for each vertex, if an edge doesn't exists between itself and its successor, return false
        for i in range(len(path) - 1):
            if self.adj_matrix[path[i]][path[i+1]] == 0:
                return False
        # otherwise return true
        return True

    def dfs(self, v_start, v_end=None) -> []:
        """
        Performs a depth-first search (DFS) in the graph and returns a list of vertices
        visited during the search, in the order they were visited
        """
        # get all vertices
        vertices = self.get_vertices()

        # if the starting vertex is not in the graph, return an empty list
        if v_start not in vertices:
            return []

        # if the ending vertex is not in the graph, ignore the ending vertex
        if v_end not in vertices:
            v_end = None

        # create lists
        visited = []    # result visited list
        stack = []      # stack to hold vertices

        # start DFS search
        return self.dfs_helper(v_start, v_end, visited, stack)

    def dfs_helper(self, v_cur, v_end, visited, stack):
        """
        Recursive helper function for dfs method
        """
        # get the edges
        edges = self.get_edges()

        # add the current vertex to visited
        visited.append(v_cur)

        # if the current vertex reached at the ending vertex, return visited
        if v_cur == v_end:
            return visited

        # temp list to save available edges and to sort before adding to stack
        temp = []
        for e in edges:
            if e[0] == v_cur and e[1] not in visited:
                temp.append(e[1])
        temp = sorted(temp)

        # if edge is already in stack, remove
        for t in temp:
            if t in stack:
                stack.remove(t)

        # add the available edges to stack
        for i in range(len(temp) - 1, -1, -1):
            stack.append(temp[i])

        # call next recursive funtion with an element in stack
        if stack:
            v_next = stack.pop()
            return self.dfs_helper(v_next, v_end, visited, stack)
        else:
            return visited

    def bfs(self, v_start, v_end=None) -> []:
        """
        Performs a depth-first search (BFS) in the graph and returns a list of vertices
        visited during the search, in the order they were visited
        """
        # get all vertices
        vertices = self.get_vertices()

        # if the starting vertex is not in the graph, return an empty list
        if v_start not in vertices:
            return []

        # if the ending vertex is not in the graph, ignore the ending vertex
        if v_end not in vertices:
            v_end = None

        # create lists
        visited = []    # result visited list
        queue = deque()      # queue to hold vertices

        # start BFS search
        return self.bfs_helper(v_start, v_end, visited, queue)

    def bfs_helper(self, v_cur, v_end, visited, queue):
        """
        Recursive helper function for bfs method
        """
        # get the edges
        edges = self.get_edges()

        # add the current vertex to visited
        visited.append(v_cur)

        # if the current vertex reached at the ending vertex, return visited
        if v_cur == v_end:
            return visited

        # add all edges to queue if not visited and not in the queue yet
        for e in edges:
            if e[0] == v_cur and e[1] not in visited and e[1] not in queue:
                queue.append(e[1])

        # call next recursive funtion with an element in stack
        if queue:
            v_next = queue.popleft()
            return self.bfs_helper(v_next, v_end, visited, queue)
        else:
            return visited

    def has_cycle(self):
        """
        Returns True if there is at least one cycle in the graph. 
        If the graph is acyclic, the method returns False.
        """
        vertices = self.get_vertices()

        # create a set to track visited set with boolean values
        # if visited once, it will remain marked as True        
        visited = {}
        for i in vertices:
            visited[i] = False

        # create a set to track current stack
        # once disconnected, it will mark back to False
        stack = {}  
        for i in vertices:
            stack[i] = False

        # iterate over the vertices to check for cycle
        for v in range(self.v_count):
            if visited[v] == False:
                if self.has_cycle_helper(v,visited,stack) == True: 
                    return True

        return False

    def has_cycle_helper(self, v, visited, stack): 
        """
        Recursive helper function for has_cycle_helper method
        """
        # set visited and stack status to True
        visited[v] = True
        stack[v] = True

        for j in range(self.v_count):
            # if no edges, continue
            if self.adj_matrix[v][j] == 0:
                continue
            # if there is an edge vertex that hasn't been visited, go check
            if visited[j] == False:                 
                if self.has_cycle_helper(j, visited, stack) == True: 
                    return True
                # current vertex has no where to go, go back to parent node
                else: 
                    parent = v
                    if self.has_cycle_helper(parent, visited, stack) == True:
                        return True
                    else:
                        continue
            elif visited[j] == True and stack[j] == True: 
                return True
  
        # at this point, no edges found for the given vertex 
        # mark the stack back to false
        stack[v] = False  

    def dijkstra(self, src: int) -> []:
        """
        Implements the Dijkstra algorithm to compute the length of the shortest path
        from a given vertex to all other vertices in the graph. It returns a list with
        minimum distance for all vertices from SRC.
        If a certain vertex is not reachable from SRC, returned INFINITY (float(‘inf’)).
        """
        infinity = float('inf')
        vertices = self.get_vertices()

        # track shortest distance for each vertex
        shortest_dist = {}
        for i in vertices:
            shortest_dist[i] = infinity

        # set the src distance to 0
        shortest_dist[src] = 0

        # boolean set to track if the vertex is in the shortest path
        is_in_path = {}
        for i in vertices:
            is_in_path[i] = False

        for i in range(self.v_count):
            # find minimum distance vertex
            min_v = self.dijkstra_helper(shortest_dist, is_in_path)
            is_in_path[min_v] = True

            # find the next vertex and update distance
            j = min_v
            for i in range(self.v_count):
                if is_in_path[i]:
                    pass
                # find next vertex i where self.adj_matrix[j][i] has a weight
                elif self.adj_matrix[j][i] > 0:
                    # find shortest value
                    # adding the distance up to the current v j and the weight at the next v
                    if shortest_dist[i] > shortest_dist[j] + self.adj_matrix[j][i]:
                        shortest_dist[i] = shortest_dist[j] + self.adj_matrix[j][i]
        
        # get the list of values
        result = list(shortest_dist.values())
        return result

    def dijkstra_helper(self, shortest_dist, is_in_path):
        """
        Helper function for dijkstra method
        """
        # minimum distance for next vertex
        min = float('inf')

        # find mininum distance vertex
        min_v = 0
        for i in range(self.v_count):
            if shortest_dist[i] < min and is_in_path[i] == False:
                min = shortest_dist[i]
                min_v = i
        return min_v


if __name__ == '__main__':
    
    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = DirectedGraph()
    print(g)
    for _ in range(5):
        g.add_vertex()
    print(g)

    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    for src, dst, weight in edges:
        g.add_edge(src, dst, weight)
    print(g)


    print("\nPDF - method get_edges() example 1")
    print("----------------------------------")
    g = DirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    test_cases = [[0, 1, 4, 3], [1, 3, 2, 1], [0, 4], [4, 0], [], [2]]
    for path in test_cases:
        print(path, g.is_valid_path(path))


    print("\nPDF - method dfs() and bfs() example")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g)
    for start in range(5):
        print(f'{start} DFS:{g.dfs(start)} BFS:{g.bfs(start)}')


    print("\nPDF - method bfs() example 1")
    print("--------------------------------------")
    edges = [(1, 8, 1), (3, 7, 3), (3,10,8), (4,12,15), (5,2,16),
    (5,3,5), (5,8,17), (7,2,3), (7,5,6), (10,2,10), (11,0,4), (11,9,7), (12,7,17), (12,9,15)]
    g = DirectedGraph(edges)
    print(g)
    for start in range(3,4):
        print(f'{start} BFS:{g.bfs(start)}')
    # [3, 7, 10, 2, 5, 8]

    print("\nPDF - method bfs() example 2")
    print("--------------------------------------")
    edges = [(0,11,3),(2, 3, 14), (2,8,2), (3,7,14), (5,2,10), (5,9,16),
    (7,11,3), (8,11,5), (9,1,17), (9,2,20), (9,11,8), (10,9,10), (11,12,20), (12,2,6)]
    g = DirectedGraph(edges)
    print(g)
    for start in range(5,6):
        print(f'{start} BFS:{g.bfs(start)}')
    # [5, 2, 9, 3, 8, 1, 11, 7, 12]
    
    print("\nPDF - method bfs() example 3")
    print("--------------------------------------")
    edges = [(0, 1, 3), (0, 7, 5), (2, 10, 16), (3,0,2), (4,6,20), (4,11,13),
    (5,6,5), (5,8,13), (7,12,5), (9,4,17), (12,1,17), (12,2,5)]
    g = DirectedGraph(edges)
    print(g)
    for start in range(0, 1):
        print(f'{start} BFS:{g.bfs(start)}')
    # [0, 1, 7, 12, 2, 10]
    
    print("\nPDF - method dfs() example 1")
    print("--------------------------------------")
    edges = [(2, 5, 7), (2,8,15), (3,1,11), (4,0,18), (6,10,18),
    (7,0,4), (7,2,15), (7,5,13), (7,6,11), (9,12,15), (11,3,5)]
    g = DirectedGraph(edges)
    print(g)
    for start in range(7, 8):
        print(f'{start} DFS:{g.dfs(start)} ')

   
    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g)
    edges_to_remove = [(3, 1), (4, 0), (3, 2)]
    for src, dst in edges_to_remove:
        g.remove_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')
    
    edges_to_add = [(4, 3), (2, 3), (1, 3), (4, 0)]
    for src, dst in edges_to_add:
        g.add_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')
    print('\n', g)
    
    
    print("\nPDF - method has_cycle() example 2")
    print("----------------------------------")
    edges = [(1,0,6), (1,9,5), (1,10,4), (2,0,3), (2,1,2), (3,1,7), 
            (5,2,8), (5,6,13), (5,9,18), (6,2,19), (7,2,7), (7,10,4), (11,12,7)]
    g = DirectedGraph(edges)
    print(g)
    print(g.has_cycle(), sep='\n')
    
    
    print("\nPDF - dijkstra() example 1")
    print("--------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    g.remove_edge(4, 3)
    print('\n', g)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    """
    
