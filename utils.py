import os
import sys

sys.path.append(os.path.abspath(__file__))
from a_star_search_algo import *

total_cell_n = 51
def get_visible_ghosts(ghost_locs, maze):
  visible_ghosts = []
  for ghost in ghost_locs:
    row = int(ghost_loc/total_cell_n)
    col = int(ghost_loc%total_cell_n)
    if(maze[row][col] == 1):
      print("Ghost ",ghost, " is an unvisible ghost")
      continue;
    visible_ghosts.append(ghost)
  return visible_ghosts

def choose_goal_oriented_choice(choices, neighbor_data):
  end_goal = total_cell_n*total_cell_n-1
  minm = 2602
  agent_choice = None
  for choice in choices:
    path = a_star_search_algo(choice, end_goal, neighbor_data)
    #print("Choice path is ", path)
    if(path and len(path) < minm):
      minm = len(path)
      agent_choice = choice
  print("Based on goal orientation, agent choice would be ", agent_choice)
  return agent_choice

def choose_survival_oriented_choice(path,ghost_locs):
  total_cost = 0
  for ghost_loc in ghost_locs:
    minm = 2602
    for path_cell in path:
      dist = find_distance(ghost_loc, path_cell)
      if(dist < minm):
        minm = dist
    total_cost = total_cost + minm
  return total_cost/len(ghost_locs)

def get_nearest_visible_ghost(maze, agent_loc, ghost_locs, neighbor_data):
  minm = 2602 # Visiting each and every cell in maze
  nearest_ghost = None
  connecting_path = []
  print("Agent location is ", agent_loc)
  for ghost_loc in ghost_locs:
    ghost_row = int(ghost_loc/total_cell_n)
    ghost_col = int(ghost_loc%total_cell_n)
    print("Ghost row & col are:", ghost_row, ghost_col)
    if maze[ghost_row][ghost_col] == 1:
      # This ghost isnt visible
      continue
    print("Ghost loc is ", ghost_loc)
    path = a_star_search_algo(agent_loc, ghost_loc, neighbor_data)
    if len(path)<minm:
      minm = len(path)
      nearest_ghost = ghost_loc
  return nearest_ghost
