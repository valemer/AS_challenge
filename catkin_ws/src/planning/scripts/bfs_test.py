from collections import deque
import time
import random

# BFS from given source s
def bfs(adj, s):
  
    # Create a queue for BFS
    q = deque()
    
    # Initially mark all the vertices as not visited
    # When we push a vertex into the q, we mark it as 
    # visited
    visited = [False] * len(adj)

    # Mark the source node as visited and enqueue it
    visited[s] = True
    q.append(s)

    # Iterate over the queue
    while q:
      
        # Dequeue a vertex from queue and print it
        curr = q.popleft()
        print(curr, end=" ")

        # Get all adjacent vertices of the dequeued 
        # vertex. If an adjacent has not been visited, 
        # mark it visited and enqueue it
        for x in adj[curr]:
            if not visited[x]:
                visited[x] = True
                q.append(x)

# Function to add an edge to the graph
def add_edge(adj, u, v):
    adj[u].append(v)
    adj[v].append(u)

# Example usage
if __name__ == "__main__":
  
    # Number of vertices in the graph
    V = 1000000

    # Adjacency list representation of the graph
    adj = [[] for _ in range(V)]

    # Add edges to form a straight path
    for i in range(V - 1):
        add_edge(adj, i, i + 1)

    # Perform BFS traversal starting from vertex 0
    print("BFS starting from 0: ")
    start_time = time.time()
    bfs(adj, 0)
    end_time = time.time()
    print(f"\nTime taken: {end_time - start_time} seconds")