import copy
import numpy as np
import random
import sys
import os
total_cell_n = 51

sys.path.append(os.path.abspath(__file__))
from search_algos import *

class The_Environment():
  def __init__(self, ghost_count=25):
    self.maze = np.zeros((total_cell_n, total_cell_n))
    self.path = []
    self.neighbor_data = {}
    self.ghost_locs = []
    self.ghost_count = ghost_count
    self.the_environment()
  
  def create_maze(self):
    # Creates 51*51 maze; 1 - value for blocked cells and 0 - unblocked cells
    self.maze = np.zeros((total_cell_n, total_cell_n))
    for row in range(len(self.maze)):
      for col in range(len(self.maze[row])):
        if random.random() <= 0.28:
          self.maze[row][col] = 1
    return self.maze

  def remove_entry_exit_blocks(self):
    # Unblocking top & bottom cells of maze if at all they are blocked
    self.maze[0][0] = 0
    self.maze[total_cell_n-1][total_cell_n-1] = 0

  def neighbor_map(self):
    # Creates and returns a data structure of format {cell_no:[list of neighbors of the cell]}
    m = len(self.maze)
    n = len(self.maze[0])
    neighbor_data = {}
    for row in range(m):
      for col in range(n):
        cell = row*total_cell_n + col
        neighbor_data[cell] = []
        if(row!=0 and self.maze[row-1][col] == 0):
            neigh = (row-1)*total_cell_n + col
            neighbor_data[cell].append(neigh)
        if(col!=0 and self.maze[row][col-1] == 0):
            neigh = row*total_cell_n + col - 1
            neighbor_data[cell].append(neigh)
        if(row!=m-1 and self.maze[row+1][col] == 0):
            neigh = (row+1)*total_cell_n + col
            neighbor_data[cell].append(neigh)
        if(col!=n-1 and self.maze[row][col+1] == 0):
            neigh = row*total_cell_n + col + 1
            neighbor_data[cell].append(neigh)
    return neighbor_data

  def generate_ghosts(self):
    count = 0
    ghost_locs = []
    while count < self.ghost_count:
      cell = np.random.randint(0, total_cell_n*total_cell_n-1)
      if get_existing_path(0, cell, self.neighbor_data):
        ghost_locs.append(cell)
        count = count + 1
    print("Location of ghosts at birth : ",ghost_locs)
    return ghost_locs
  
  # generate locations for ghost 
  def manage_ghost_location(self, curr_locations, neighbor_data):
    new_loc=[]
    for ghost_loc in curr_locations:
      #print("ghost locations ", ghost_loc)
      row = int(ghost_loc/total_cell_n)
      col = int(ghost_loc%total_cell_n)
      #print("Row & col are ", row, col)
      movements = []
      if (row > 0):
        movements.append((row-1)*total_cell_n + col)
      if (col > 0):
        movements.append(row*total_cell_n + col-1)
      if (row < total_cell_n-1):
        movements.append((row+1)*total_cell_n + col)
      if (col < total_cell_n-1):
        movements.append(row*total_cell_n + col+1)
      #print("Ghost movements :", movements)
      loc = random.choice(movements)
      movements = [loc]
      #print("Ghost choice :", movements)
      if loc in neighbor_data[ghost_loc] :
        new_loc.append(loc)
      else :
        movements.append(ghost_loc)
        new_loc.append(random.choice(movements))
    return new_loc

  def the_environment(self):
    # Creates and returns the valid maze
    print("The Great Environment started")
    self.maze = self.create_maze()
    unique, counts = np.unique(self.maze, return_counts=True)
    print("Count is: ", dict(zip(unique, counts)))
    self.remove_entry_exit_blocks()
    self.neighbor_data = self.neighbor_map()
    print("Neighboring data is ", self.neighbor_data)
    self.path = get_existing_path(0, total_cell_n*total_cell_n-1, self.neighbor_data)
    if self.path:
      self.ghost_locs = self.generate_ghosts()
      print("Path exists from source to destination")
    else:
      print("No Path exists from source to destination")
      return self.the_environment()
    print("Ghosts Locations are", self.ghost_locs)
    print("Initial path obtained for agent is -- > ", self.path)

def is_ghost_born_in_top_left_cell(ghost_locs):
  top_cell_no = 0
  if top_cell_no in ghost_locs:
    return True
  return False

def main():
  env = The_Environment()
  unique, counts = np.unique(env.maze, return_counts=True)
  print("Env ghosts ", env.ghost_locs)
  print(env.maze)
  if is_ghost_born_in_top_left_cell(env.ghost_locs):
    print("All agents will die")

if __name__ == '__main__':
  main()
