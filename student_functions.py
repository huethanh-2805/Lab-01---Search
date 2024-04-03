import numpy as np
import queue

def BFS(matrix, start, end):
    """
    BFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    # TODO: 
   
    path=[]
    visited={}
    temp = {}
    n = len(matrix) #Số đỉnh của ma trận
    frontier = queue.Queue() #Khởi tạo hàng đợi để thêm vào hàng đợi các node kề với node đang xét
    frontier.put(start)
    trace = [] #Lưu vết các đỉnh đã thăm

    while not frontier.empty():
        current_node = frontier.get()
        trace.append(current_node)

        if current_node == start:
            visited[current_node] = -1
        else:
            visited[current_node] = temp[current_node]

        #Thêm các đỉnh kề với đỉnh đang xét vào hàng đợi
        for i in range(n):
            if matrix[current_node][i] != 0:
                adjacent_node = i
                if adjacent_node not in trace:
                    frontier.put(adjacent_node)
                    # Kiểm tra nút cha của adjacent_node đã được lưu chưa
                    if adjacent_node not in temp:
                        temp[adjacent_node] = current_node # Lưu nút cha của adjacent_node

        if current_node == end: 
            break
                     
    #Truy vết đường đi        
    while current_node is not -1:
        path.append(current_node)
        current_node = visited[current_node]
    path.reverse()

    print(visited)
    print(path)

    return visited, path

def DFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO: 

    path=[]
    visited={}
    temp = {}
    n = len(matrix) #Số đỉnh của ma trận
    frontier = [] #Khởi tạo stack để thêm vào các node kề với node đang xét 
    frontier.append(start)

    while len(frontier) > 0:
        current_node = frontier.pop()

        if current_node == start:
            visited[current_node] = -1
        else:
            visited[current_node] = temp[current_node]

        #Thêm các đỉnh kề với đỉnh đang xét vào stack
        for i in range(n):
            if matrix[current_node][i] != 0:
                adjacent_node = i
                if adjacent_node not in visited:
                    frontier.append(adjacent_node)
                    temp[adjacent_node] = current_node # Lưu nút cha của adjacent_node

        if current_node == end: 
            break

    #Truy vết đường đi        
    while current_node != -1:
        path.append(current_node)
        current_node = visited[current_node]
    path.reverse()

    print(visited)
    print(path)
   
    return visited, path


def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:  
    path=[]
    visited={}
    n = len(matrix) #Số đỉnh của ma trận
    frontier = queue.PriorityQueue() #Khởi tạo hàng đợi ưu tiên để thêm vào các node kề với node đang xét
    frontier.put((0, start, -1)) # Thêm node start vào hàng đợi ưu tiên với cost là 0
    trace = [] #Lưu vết các đỉnh đã thăm

    while not frontier.empty():
        current_cost, current_node, current_parent = frontier.get()
        if current_node not in trace: #Để tránh trường hợp node đó đã ghé thăm mà có cost nhỏ nhất trong frontier hiện tại
            trace.append(current_node)
            visited[current_node] = current_parent

        if current_node == end: 
            break

        #Thêm các đỉnh kề với đỉnh đang xét vào hàng đợi
        for i in range(n):
            if matrix[current_node][i] != 0:
                adjacent_node = i
                if adjacent_node not in trace:
                    adjacent_cost = current_cost + matrix[current_node][i]
                    found = False
                    for item in frontier.queue:
                        if item[1] == adjacent_node:
                            found = True
                            #Thay thế nếu chi phí nhỏ hơn
                            if adjacent_cost < item[0]:
                                frontier.queue.remove(item)
                                frontier.put((adjacent_cost, adjacent_node, current_node)) 
                            break

                    if not found:
                        frontier.put((adjacent_cost, adjacent_node, current_node))
                     
    #Truy vết đường đi        
    while current_node is not -1:
        path.append(current_node)
        current_node = visited[current_node]
    path.reverse()

    print(visited)
    print(path)
    
    return visited, path


def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm 
    heuristic : edge weights
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
   
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
    path=[]
    visited={}
    n = len(matrix) #Số đỉnh của ma trận
    frontier = queue.PriorityQueue() #Khởi tạo hàng đợi ưu tiên để thêm vào các node kề với node đang xét
    frontier.put((0, start, -1)) # Thêm node start vào hàng đợi ưu tiên với cost là 0
    trace = [] #Lưu vết các đỉnh đã thăm

    while not frontier.empty():
        current_cost, current_node, current_parent = frontier.get()
        if current_node not in trace: #Để tránh trường hợp node đã ghé thăm mà có cost nhỏ nhất trong frontier hiện tại
            trace.append(current_node)
            visited[current_node] = current_parent

        if current_node == end: 
            break

        #Thêm các đỉnh kề với đỉnh đang xét vào hàng đợi
        for i in range(n):
            if matrix[current_node][i] != 0:
                adjacent_node = i
                if adjacent_node not in trace:
                    adjacent_cost = matrix[current_node][i]
                    found = False
                    for item in frontier.queue:
                        if item[1] == adjacent_node:
                            found = True
                            #Thay thế nếu chi phí nhỏ hơn
                            if adjacent_cost < item[0]:
                                frontier.queue.remove(item)
                                frontier.put((adjacent_cost, adjacent_node, current_node)) 
                            break

                    if not found:
                        frontier.put((adjacent_cost, adjacent_node, current_node))
                            
    #Truy vết đường đi        
    while current_node is not -1:
        path.append(current_node)
        current_node = visited[current_node]
    path.reverse()

    print(visited)
    print(path)
    return visited, path

def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
    heuristic: eclid distance based positions parameter
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 

    path=[]
    visited={}
    n = len(matrix)
    frontier = queue.PriorityQueue()
    frontier.put((0, start, -1))
    trace = []

    while not frontier.empty():
        current_cost, current_node, current_parent = frontier.get()
        if current_node not in trace:
            trace.append(current_node)
            visited[current_node] = current_parent

        if current_node == end:
            break

        for i in range(n):
            if matrix[current_node][i] != 0:
                adjacent_node = i
                if adjacent_node not in trace:
                    # heuristic: Euclidean distance between current position and goal position
                    heuristic_cost = ((pos[end][0] - pos[adjacent_node][0]) ** 2 + (pos[end][1] - pos[adjacent_node][1]) ** 2) ** 0.5
                    adjacent_cost = current_cost + matrix[current_node][i] + heuristic_cost
                    found = False
                    for item in frontier.queue:
                        if item[1] == adjacent_node:
                            found = True
                            #Thay thế nếu chi phí nhỏ hơn
                            if adjacent_cost < item[0]:
                                frontier.queue.remove(item)
                                frontier.put((adjacent_cost, adjacent_node, current_node)) 
                            break

                    if not found:
                        frontier.put((adjacent_cost, adjacent_node, current_node))

    while current_node is not -1:
        path.append(current_node)
        current_node = visited[current_node]
    path.reverse()

    print(visited)
    print(path)
    return visited, path

