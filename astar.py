import heapq

class Node:
    def __init__(self, name, parent=None, g=0, h=0):
        self.name = name  # Node name (can be a coordinate, or any identifier)
        self.parent = parent  # Parent node in the path
        self.g = g  # Cost from the start node to this node
        self.h = h  # Heuristic estimated cost to the goal
        self.f = g + h  # f = g + h (total estimated cost)
    
    def __lt__(self, other):
        return self.f < other.f  # This is used by the priority queue (heapq) to compare nodes


def astar(start, goal, graph, heuristic):
    open_list = []
    closed_list = set()
    
    # Initialize the start node
    start_node = Node(start, g=0, h=heuristic[start])
    heapq.heappush(open_list, start_node)
    
    while open_list:
        # Get the node with the lowest f-value
        current_node = heapq.heappop(open_list)
        
        # If we reached the goal, reconstruct the path
        if current_node.name == goal:
            path = []
            while current_node:
                path.append(current_node.name)
                current_node = current_node.parent
            return path[::-1]  # Reverse the path to get it from start to goal
        
        closed_list.add(current_node.name)
        
        # Get neighbors of the current node
        for neighbor, cost in graph[current_node.name].items():
            if neighbor in closed_list:
                continue
            
            g_cost = current_node.g + cost
            h_cost = heuristic.get(neighbor, float('inf'))
            
            # Create a new node for the neighbor
            neighbor_node = Node(neighbor, parent=current_node, g=g_cost, h=h_cost)
            
            # Add the neighbor to the open list
            heapq.heappush(open_list, neighbor_node)
    
    return None  # Return None if there is no path


# Example graph and heuristic:
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'D': 2, 'E': 5},
    'C': {'A': 4, 'E': 1},
    'D': {'B': 2, 'E': 3},
    'E': {'B': 5, 'C': 1, 'D': 3}
}

# Heuristic values (estimated cost from each node to the goal)
heuristic = {
    'A': 7,
    'B': 6,
    'C': 2,
    'D': 1,
    'E': 0
}

# Example: Find the shortest path from 'A' to 'E'
start = 'A'
goal = 'E'
path = astar(start, goal, graph, heuristic)

print(f"Path from {start} to {goal}: {path}")
