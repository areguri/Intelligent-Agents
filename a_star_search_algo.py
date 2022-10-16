import os
import sys
import copy
sys.path.append(os.path.abspath(__file__))

total_cell_n = 51
def find_distance(cell_a, cell_b):
  a_row = cell_a/total_cell_n
  a_col = cell_a%total_cell_n
  b_row = cell_b/total_cell_n
  b_col = cell_b%total_cell_n
  return abs(a_row - b_row) + abs(a_col - b_col)

def set_heuristic(h, start_cell, goal_cell):
  for cell in range(start_cell, goal_cell+1):
      h[cell] = find_distance(cell, goal_cell)

def a_star_search_algo(start_cell, goal_cell, neighbor_data, ghost_locs=None):
  g = {}
  h = {}
  parent = {}
  closed_list = set()
  open_list = set()
  open_list.add(start_cell)
  g[start_cell] = 0
  parent[start_cell] = -1
  path = []
  set_heuristic(h, 0, total_cell_n*total_cell_n-1)
  local_neighbor_data = copy.deepcopy(neighbor_data)
  if ghost_locs:
    for ghost_loc in ghost_locs:
      for neigh in local_neighbor_data:
        if(ghost_loc in local_neighbor_data[neigh]):
          #print("Removing ", ghost_loc, " from neighbor ", neigh)
          local_neighbor_data[neigh].remove(ghost_loc)

  while(len(open_list) > 0):
      curr_cell = -1
      for cell in open_list:
          if(curr_cell == -1 or g[cell] + h[cell] <= g[curr_cell] + h[curr_cell]):
              curr_cell = cell
      if curr_cell == -1:
          return path
      if(curr_cell == goal_cell):
          while(parent[curr_cell] != -1):
              path.append(curr_cell)
              curr_cell = parent[curr_cell]
          path.append(curr_cell)
          path.reverse()
          return path

      for neigh in local_neighbor_data[curr_cell]:
          if (neigh not in open_list) and (neigh not in closed_list):
              open_list.add(neigh)
              parent[neigh] = curr_cell
              g[neigh] = g[curr_cell] + 1
          elif(neigh in open_list) or (neigh in closed_list):
              if(g[neigh] > g[curr_cell] + 1):
                  parent[neigh] = curr_cell
                  g[neigh] = g[curr_cell] + 1
                  if neigh in closed_list:
                      closed_list.remove(neigh)
                      open_list.add(neigh)

      open_list.remove(curr_cell)
      closed_list.add(curr_cell)
  return path
