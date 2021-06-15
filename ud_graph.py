# Course: CS261 - Data Structures
# Author: Hye Yeon Park
# Assignment: Assignment 6: Graph Implementation
# Description: UndirectedGraph class, designed to support the following type of graph:
#       undirected, unweighted, no duplicate edges, no loops. Cycles are allowed.
#       Undirected graphs is stored as a Python dictionary of lists where keys are
#       vertex names(strings) and associated values are Python lists with names(in any
#       order) of vertices connected to the 'key' vertex.
#       e.g.) self.adj_list = {'A': ['B', 'C'], 'B': ['A', 'C', 'D'], 'C': ['B', 'A'], 'D': ['B']}
#       This implementation includes the following methods:
#               add_vertex(), add_edge()
#               remove_edge(), remove_vertex()
#               get_vertices(), get_edges()
#               is_valid_path(), dfs(), bfs()
#               count_connected_components(), has_cycle()

import heapq
from collections import deque


class UndirectedGraph:
    """
    Class to implement undirected graph
    - duplicate edges not allowed
    - loops not allowed
    - no edge weights
    - vertex names are strings
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency list
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.adj_list = dict()

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            for u, v in start_edges:
                self.add_edge(u, v)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        out = [f'{v}: {self.adj_list[v]}' for v in self.adj_list]
        out = '\n  '.join(out)
        if len(out) < 70:
            out = out.replace('\n  ', ', ')
            return f'GRAPH: {{{out}}}'
        return f'GRAPH: {{\n  {out}}}'

    # ------------------------------------------------------------------ #

    def add_vertex(self, v: str) -> None:
        """
        Add new vertex to the graph
        """
        # if the vertex is not in the dictionary(adjacency list)
        if v not in self.adj_list:
            # add it as key with empty list(value)
            self.adj_list[v] = []

    def add_edge(self, u: str, v: str) -> None:
        """
        Add edge to the graph
        """
        # if u and v refer to the same vertex, the method does nothing
        if u == v:
            return

        # if vertex u, v do not exist in the graph
        # first add the vertices
        if u not in self.adj_list:
            self.add_vertex(u)
        if v not in self.adj_list:
            self.add_vertex(v)

        # add egdes for each vertex if it is not listed yet
        if v not in self.adj_list[u]:
            self.adj_list[u].append(v)
        if u not in self.adj_list[v]:
            self.adj_list[v].append(u)

    def remove_edge(self, v: str, u: str) -> None:
        """
        Remove edge from the graph
        """
        # If passed vertices v, u do not exist in the graph, the method does nothing
        if u not in self.adj_list or v not in self.adj_list:
            return

        # if there is no edge between them, the method does nothing
        if u not in self.adj_list[v] or v not in self.adj_list[u]:
            return

        # otherwise remove the edge for each vertex
        self.adj_list[v].remove(u)
        self.adj_list[u].remove(v)

    def remove_vertex(self, v: str) -> None:
        """
        Remove vertex and all connected edges
        """
        # If the given vertex does not exist, the method does nothing
        if v not in self.adj_list:
            return

        # remove the given vertex(key) from the graph dictionary
        self.adj_list.pop(v)

        # remove edges including v from all other vertices
        for vertex in self.adj_list:
            edges = self.adj_list[vertex]
            if v in edges:
                edges.remove(v)

    def get_vertices(self) -> []:
        """
        Return list of vertices in the graph (any order)
        """
        result = []
        for v in self.adj_list:
            result.append(v)
        return result

    def get_edges(self) -> []:
        """
        Return list of edges in the graph (any order)
        """
        # list of edges
        result = []
        # list of vertices already added to result list of edges
        already_added = []

        # for all vertices, create a tuple pair with each edge
        for v in self.adj_list:
            edges = self.adj_list[v]
            if edges:
                for i in range(len(edges)):
                    if edges[i] not in already_added:
                        result.append((v, edges[i]))
                        # save the added vertex to avoid duplicates
                        already_added.append(v)
        return result

    def is_valid_path(self, path: []) -> bool:
        """
        Return true if provided path is valid, False otherwise
        """
        # if the given path is empty, return True
        if len(path) == 0:
            return True
        # if the path has one vertex listed in the graph
        if len(path) == 1:
            if path[0] in self.adj_list:
                return True
            else:
                return False

        v = path[0]
        # get the list of edges of the vertex
        edges = self.adj_list[v]
        # if the successor vertex is in the edges list
        # remove the first vertex from the path and repeat the process
        if path[1] in edges:
            path = path[1:]
            return self.is_valid_path(path)
        else:
            return False

    def dfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during DFS search
        Vertices are picked in alphabetical order
        """
        # if the starting vertex is not in the graph, return an empty list
        if v_start not in self.adj_list:
            return []

        # if the ending vertex is not in the graph, ignore the ending vertex
        if v_end not in self.adj_list:
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
        # add the current vertex to visited
        visited.append(v_cur)

        # if the current vertex reached at the ending vertex, return visited
        if v_cur == v_end:
            return visited

        # base case to end recursive function
        # sort the edges for the vertex
        edges = sorted(self.adj_list[v_cur])
        if len(visited) > 1 and not stack:
            finished = True
            for i in range(len(edges)):
                if edges[i] not in visited:
                    finished = False
            if finished:
                return visited

        # only save edges from previous vertex if the edge is not in the current edge
        temp = []
        for i in range(len(stack)):
            if stack[i] not in edges:
                temp.append(stack[i])
        stack = temp

        # push all edges to the stack in reverse order
        for j in range(len(edges)-1, -1, -1):
            if edges[j] not in stack and edges[j] not in visited:
                stack.append(edges[j])

        # pop one edge from the stack
        if stack:
            v_cur = stack.pop()

        return self.dfs_helper(v_cur, v_end, visited, stack)

    def bfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during BFS search
        Vertices are picked in alphabetical order
        """
        # is the starting vertex is not in the graph, return an empty list
        if v_start not in self.adj_list:
            return []

        # if the ending vertex is not in the graph, ignore the ending vertex
        if v_end not in self.adj_list:
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
        # add the current vertex to visited
        visited.append(v_cur)

        # if the current vertex reached at the ending vertex, return visited
        if v_cur == v_end:
            return visited

        # base case to end recursive function
        # sort the edges for the vertex
        edges = sorted(self.adj_list[v_cur])
        if len(visited) > 1 and not queue:
            finished = True
            for i in range(len(edges)):
                if edges[i] not in visited:
                    finished = False
            if finished:
                return visited

        # push all edges to the stack
        for j in range(len(edges)):
            if edges[j] not in queue and edges[j] not in visited:
                queue.append(edges[j])

        # pop one leftmost vertex from the stack
        v_cur = queue.popleft()

        return self.bfs_helper(v_cur, v_end, visited, queue)

    def count_connected_components(self):
        """
        Return number of connected components in the graph
        """
        if len(self.adj_list) == 0:
            return 0

        # put all vertices into unvisited list
        unvisited = self.get_vertices()
        # counter for the number of connected components
        count = 0
        return self.count_connected_components_helper(count, unvisited)

    def count_connected_components_helper(self, count, unvisited):
        """
        Recursive helper function for count_connected_components method
        """
        # find visited list by calling dfs method
        visited = self.dfs(unvisited[0])
        count += 1

        # check for unvisited vertices
        while visited:
            temp = visited.pop()
            if temp in unvisited:
                unvisited.remove(temp)

        # base cases to stop recursive method
        # if all vertices are visited, return the count
        if len(unvisited) == 0:
            return count
        # if one vertex is left to visit,
        # there is one more component in the graph
        # increase count by 1 and return count
        if len(unvisited) == 1:
            count += 1
            return count

        else:
            # repeat the process on the unvisited list
            return self.count_connected_components_helper(count, unvisited)

    def has_cycle(self):
        """
        Return True if graph contains a cycle, False otherwise
        """
        vertices = self.get_vertices()
        v_count = len(vertices)

        # create a set to track visited set with boolean values
        # if visited once, it will remain marked as True
        visited = {}
        for i in range(v_count):
            visited[vertices[i]] = False

        # create a set to track current stack
        # once disconnected, it will mark back to False
        stack = {}
        for i in vertices:
            stack[i] = False

        # iterate over the vertices to check for cycle
        for i in range(v_count):
            if visited[vertices[i]] == False:
                if self.has_cycle_helper(i, visited, stack) == True:
                    return True

        return False

    def has_cycle_helper(self, i, visited, stack):
        """
        Recursive helper function for has_cycle_helper method
        """
        vertices = self.get_vertices()
        # A recursive function that uses
        # visited[] and parent to detect
        # cycle in subgraph reachable from vertex v

        # set visited and stack status to True
        visited[vertices[i]] = True
        stack[vertices[i]] = True

        # recur for all the vertices
        # adjacent to this vertex
        parent = None
        edges = self.adj_list[vertices[i]]

        for j in range(len(vertices)):
            cur = vertices[j]  # debugging
            if cur not in edges:
                continue
            # if the vertex is not visited then repeat the process
            if visited[cur] == False:
                parent = vertices[i]
                if self.has_cycle_helper(j, visited, stack) == True:
                    return True
                else:                    
                    if self.has_cycle_helper(parent, visited, stack) == True:
                        return True
                    else:
                        continue

            elif visited[cur] == True and stack[cur] == True and cur != parent:
                return True
            # if an adjacent vertex is visited and not parent of current vertex,
            # the graph has a cycle

        stack[vertices[i]] = False
        return False


if __name__ == '__main__':
    """
    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = UndirectedGraph()
    print(g)

    for v in 'ABCDE':
        g.add_vertex(v)
    print(g)

    g.add_vertex('A')
    print(g)

    for u, v in ['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE', ('B', 'C')]:
        g.add_edge(u, v)
    print(g)


    print("\nPDF - method remove_edge() / remove_vertex example 1")
    print("----------------------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    print('here')
    g.remove_vertex('DOES NOT EXIST')
    g.remove_edge('A', 'B')
    g.remove_edge('X', 'B')
    print(g)
    g.remove_vertex('D')
    print(g)


    print("\nPDF - method get_vertices() / get_edges() example 1")
    print("---------------------------------------------------")
    g = UndirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE'])
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    test_cases = ['ABC', 'ADE', 'ECABDCBE', 'ACDECB', '', 'D', 'Z']
    for path in test_cases:
        print(list(path), g.is_valid_path(list(path)))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = 'ABCDEGH'
    for case in test_cases:
        print(f'{case} DFS:{g.dfs(case)} BFS:{g.bfs(case)}')
    print('-----')
    for i in range(1, len(test_cases)):
        v1, v2 = test_cases[i], test_cases[-1 - i]
        print(f'{v1}-{v2} DFS:{g.dfs(v1, v2)} BFS:{g.bfs(v1, v2)}')


    print("\nPDF - method count_connected_components() example 1")
    print("---------------------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print(g.count_connected_components(), end=' ')
    print()

    """
    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG',
        'add FG', 'remove GE')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print('{:<10}'.format(case), g.has_cycle())
