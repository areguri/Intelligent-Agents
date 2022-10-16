import os
import sys

sys.path.append(os.path.abspath(__file__))

def bfs(neighbor_data, queue, visited, prev):
    start = queue.pop(0)
    for neighbor in neighbor_data[start]:
        if neighbor not in visited:
            visited.add(neighbor)
            prev[neighbor] = start
            queue.append(neighbor)

def is_intersecting(start_v, end_v):
    return start_v.intersection(end_v)

def get_path(start_prev, end_prev, start_cell, end_cell, intersection_cell):
    #Returns the path for bi-directional bfs algorithm based on intersection node
    path = []
    path.append(intersection_cell)
    forward_cell = intersection_cell
    backward_cell = intersection_cell
    while(forward_cell != start_cell):
      forward_cell = start_prev[forward_cell]
      path.append(forward_cell)
    path.reverse()
    while(backward_cell != end_cell):
      backward_cell = end_prev[backward_cell]
      path.append(backward_cell)
    return path

def bi_directional_bfs(start_cell, end_cell,neighbor_data):  
  if (len(neighbor_data[start_cell]) == 0 or len(neighbor_data[end_cell]) == 0):
      return False
  start_queue = [start_cell]
  end_queue = [end_cell]
  start_visited = set()
  start_visited.add(start_cell)
  end_visited = set()
  end_visited.add(end_cell)
  start_prev = {}
  start_prev[start_cell] = -1
  end_prev = {}
  end_prev[end_cell] = -1
  path = []
  while(start_queue and end_queue):
    bfs(neighbor_data, start_queue, start_visited, start_prev)
    bfs(neighbor_data, end_queue, end_visited, end_prev)
    intersection_cell = is_intersecting(start_visited, end_visited)
    if not intersection_cell:
        continue
    else:
        inter_cell = list(intersection_cell)[0]
        path = get_path(start_prev, end_prev, start_cell, end_cell, inter_cell)
        return path
  return path

def get_existing_path(start_cell, end_cell, neighbor_data):
    return  bi_directional_bfs(start_cell , end_cell,neighbor_data)