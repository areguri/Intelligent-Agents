import argparse
import os
import sys
import numpy as np
sys.path.append(os.path.abspath(__file__))

from utils import *
from environment import *
from search_algos import *
from a_star_search_algo import *

total_cell_n = 51
end_goal = total_cell_n*total_cell_n-1

class Agent(The_Environment):
  def __init__(self):
    pass

  def agent_1(self, maze, path, neighbor_data, ghost_locs):

      '''Agent 1 follows the shortest path passed in the function 
         argument ignoring the environment. This path is fetched 
         through bi-directional BFS algorithm when generated the
         environment
         maze - Environment maze
         path - Bi-Directional BFS generated shortest path
         neighbor_data - A dictionary Data Structure where keys are 
         cell numbers and values are list of neigbor cell numbers.
         Example: 
         {0:[1, 51]} - For cell number 0 & 51 are neighbors
         ghost_locs - List containing Locations of Ghosts
         '''

      agent_loc = path[0]
      index = 0
      print("Agent 1 started")
      while agent_loc != path[-1]: 
        print("Ghost Locations are: ", ghost_locs)
        print("Agent loc is ", agent_loc)
        if agent_loc in ghost_locs : 
          # Ghost entering agent's cell scenario. Agent dies.
          print("Ghost entered agent's cell. Agent died")
          return False
        else:
          index = index + 1
          agent_loc = path[index]
          if agent_loc in ghost_locs:
            # Agent entering Ghost's cell scenario. Agent dies.
            print("Agent entered Ghost cell. Agent died")
            return False
          if(agent_loc == end_goal):
            print("Agent reached the goal state")
            return True
        ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)
      print("Agent reached the goal state")
      return True

  def agent_2(self, maze, path, neighbor_data, ghost_locs, prev_cell=None, agent_simulated=False):

    '''Agent 2 looks through the maze and plans the path to the end goal considering the changes 
       in the environment. The path passed in the argument is the path generated through 
       Bi-Directional BFS. Later on at every timestamp, Agent 2 plans a new path through A_Star_Search
       algorithm and follows it. If there comes a scenario where no path exists to the end goal, 
       Agent 2 moves away from the nearest visible ghost

       maze - Environment maze
       path - Bi-Directional BFS generated shortest path
       neighbor_data - A dictionary Data Structure where keys are 
       cell numbers and values are list of neigbor cell numbers.
       Example: 
       {0:[1, 51]} - For cell number 0 & 51 are neighbors.
       ghost_locs - List containing Locations of Ghosts.
       agent_simulated would be True in case if agent_2 is simulated.
       prev_cell is valid argument in case of simulated agent 2 only
       '''

    agent_loc = path[0]
    print("Agent 2 started")
    result_path = []
    if agent_simulated:
      if agent_loc in ghost_locs:
        # Ghost existing in agent's cell scenario. Agent dies.
        print("Ghost existed in agent's cell. Agent died")
        return False
      if agent_loc != prev_cell:
        ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)
    agent_start_loc = agent_loc
    while agent_loc != end_goal: 
      result_path.append(agent_loc)
      if agent_loc in ghost_locs : 
        # Ghost entering agent's cell scenario. Agent dies.
        print(result_path)
        print("Ghost entered agent's cell. Agent died")
        return False
      else:
        # Re-planning the path to End Goal
        if(prev_cell and agent_start_loc!= prev_cell):
          path = a_star_search_algo(agent_loc, end_goal, neighbor_data, ghost_locs)
        else:
          path = a_star_search_algo(agent_loc, end_goal, neighbor_data, ghost_locs)
        print("Agent 2 re-planned path is ", path)
        if not path:
          # Agent moving away from the nearest visible ghost
          print("Current Ghost locs are ", ghost_locs)
          nearest_ghost = get_nearest_visible_ghost(maze, agent_loc, ghost_locs, neighbor_data)
          print("Nearest ghost is ", nearest_ghost)
          maxm = 0 
          farthest_neigh = None
          for neigh in neighbor_data[agent_loc]:
            ghost_agent_path = a_star_search_algo(neigh, nearest_ghost, neighbor_data)
            if len(ghost_agent_path) > maxm:
              maxm = len(ghost_agent_path)
              farthest_neigh = neigh
              agent_loc = farthest_neigh
              print("Agent moved to farthest neighbor away from ghost which is :", farthest_neigh)
        else:
          agent_loc = path[1]
          if agent_loc in ghost_locs : 
            # Agent entering Ghost's cell scenario. Agent dies.
            print("Agent entered ghost cell. Agent died")
            print(result_path)
            return False
      ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)
    print(result_path)
    print("Agent reached the goal state.")
    return True

  def agent_2_optimised(self, maze, path, neighbor_data, ghost_locs, prev_cell=None, agent_simulated=None):
    
    '''Agent 2 optimised is similar to Agent 2. The only difference is that Agent 2 doesnt re-plan the 
       path at every timestamp. If there are any obstacles(ghosts) planned in the current path, 
       then Agent 2 re-plans the path. Else it goes ahead without re-planning the path'''

    agent_loc = path[0]
    agent_move_away=0
    replans=0
    print("Agent 2 started")
    result_path = []
    if agent_simulated:
      if agent_loc in ghost_locs:
        print("Ghost entered agent's cell. Agent died")
        return False, replans, agent_move_away
      if agent_loc != prev_cell:
        ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)
    agent_start_loc = agent_loc
    while agent_loc != end_goal: 
      result_path.append(agent_loc)
      if agent_loc in ghost_locs : 
        # Ghost entering agent's cell
        print(result_path)
        print("Ghost entered agent's cell. Agent died")
        return False, replans, agent_move_away
      else:
        path_blockers = set(ghost_locs).intersection(set(path))
        # Finding if there are any path blockers in the current path
        if path_blockers or not path:
          if(prev_cell and agent_start_loc!= prev_cell):
            path = a_star_search_algo(agent_loc, end_goal, neighbor_data, ghost_locs)
          else:
            path = a_star_search_algo(agent_loc, end_goal, neighbor_data, ghost_locs)
          print("Agent 2 re-planned path is ", path)
          replans = replans + 1
          if not path:
            # Agent moving away from the nearest visible ghost.
            print("Ghost locs are ", ghost_locs)
            nearest_ghost = get_nearest_visible_ghost(maze, agent_loc, ghost_locs, neighbor_data)
            print("Agent's nearest ghost is ", nearest_ghost)
            maxm = 0 
            farthest_neigh = None
            agent_move_away = agent_move_away + 1
            for neigh in neighbor_data[agent_loc]:
              ghost_agent_path = a_star_search_algo(neigh, nearest_ghost, neighbor_data)
              if len(ghost_agent_path) > maxm:
                maxm = len(ghost_agent_path)
                farthest_neigh = neigh
                agent_loc = farthest_neigh
                print("Agent moved to farthest neighbor away from ghost which is :", farthest_neigh)
          else:
            agent_loc = path[1]
            path = path[1:]
            if agent_loc in ghost_locs : 
              # Ghost entering agent's cell
              print("Ghost entered agent's cell")
              return False, replans, agent_move_away
        else:
          print("Agent 2 doesn't need to re-plan the path. There are no path blockers")
          agent_loc = path[1]
          path = path[1:]
          if agent_loc in ghost_locs: 
            # Agent entering Ghost cell's scenario. Agent dies
            print("Agent entered ghost cell. Agent died")
            print(result_path)
            return False, replans, agent_move_away
      ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)
    print(result_path)
    print("Agent reached the goal state.")
    return True, replans, agent_move_away
  
  def agent_3(self, maze, neighbor_data, ghost_locs, start_cell=0, simulations=5):

    '''Agent 3 simulates agent 2 from each of the possible moves it can take including 
       its current position. Based on the success rates(# of times Agent 2 is alive),
       Agent 3 decides on the highest success rate.

       In case of 2 or more moves having same success rate, Agent 3 goes ahead with the 
       goal oriented behaviour(moving in the direction of the goal). 

       If there is no path to the end goal. Agent 3 moves away from the nearest 
       visible ghost, similar to Agent 2

       Default start_cell = 0, top left corner.'''

    print("Agent 3 started")
    agent_move_away=0
    while(start_cell != end_goal):
      print("Start cell is ", start_cell)
      if start_cell in ghost_locs:
        print("Ghost entered agent's cell. Agent died")
        # Ghost reached the agent
        return False, agent_move_away
      moves = neighbor_data[start_cell]
      success_rate = {}
      path = a_star_search_algo(start_cell, end_goal, neighbor_data, ghost_locs)
      if not path:
        # agent moves away from the nearest visible ghost
        print("All paths are blocked, moving away from nearest visible Ghost")
        maxm = 0
        farthest_neigh = None
        nearest_ghost = get_nearest_visible_ghost(maze, start_cell, ghost_locs, neighbor_data)
        agent_move_away = agent_move_away + 1
        for neigh in neighbor_data[start_cell]:
          ghost_agent_path = a_star_search_algo(neigh, nearest_ghost, neighbor_data)
          print("Ghost agent path is ", ghost_agent_path)
          if len(ghost_agent_path) > maxm:
            maxm = len(ghost_agent_path)
            farthest_neigh = neigh
            print("Farthest neighbor is ", farthest_neigh)
            start_cell = farthest_neigh
        print("Agent moved away from nearest possible ghost. It is now at ", farthest_neigh)
        ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)
        continue
      agent_2_ghost_locs = copy.deepcopy(ghost_locs)
      # Determining the succcess rates of every move, agent can take
      for move in moves+[start_cell]:
        agent_success = 0
        print("Determining survivability for move: ", move)
        path = a_star_search_algo(move, end_goal, neighbor_data)
        for simulation in range(simulations):
          # Agent_simulated being passed as true
          agent_2_status = self.agent_2_optimised(maze, path, neighbor_data, agent_2_ghost_locs, start_cell, agent_simulated=True)
          if agent_2_status:
            print("Woahhh, Agent 2 is alive")
            agent_success = agent_success + 1
          else:
            print("Oh no Agent 2 is ", agent_2_status)
        success_rate[move] = agent_success/simulations
      # Sorting the success rates descending
      success_rate = {k: v for k, v in sorted(success_rate.items(), key=lambda item: item[1], reverse=True)}
      print("Success rates are ", success_rate)
      agent_choices = []
      # Success rate of every move = 0 scenario. Agent continues in the same cell
      if list(success_rate.values())[0] == 0:
        print("There is no probability to win from any perspective")
        if start_cell == end_goal:
          print("Agent reached the goal")
          return True, agent_move_away
        ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)
        continue
      for key, value in success_rate.items():
        if value == list(success_rate.values())[0]:
          agent_choices.append(key)
      print("Agent Choices is ", agent_choices)
      if(len(agent_choices) == 1):
        agent_choice = agent_choices[0]
      else:
        # Choosing Goal Oriented behaviour
        agent_choice = choose_goal_oriented_choice(agent_choices, neighbor_data)
      print("Agent choice is ", agent_choice)
      start_cell = agent_choice
      if start_cell in ghost_locs:
        print("Agent entered Ghost cell. Agent died")
        # Agent reached the ghost
        return False, agent_move_away
      if start_cell == end_goal:
        print("Agent reached the goal")
        return True, agent_move_away
      ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)
    print("Agent reached the goal")
    return True, agent_move_away

  def agent_4(self, maze, neighbor_data, ghost_locs, agent_loc=0, simulations=5):
    '''Agent 4 simulates agent 2 from each of the possible moves it can take including 
       its current position. Based on the success rates(# of times Agent 2 is alive) added to the 
       avg. distance of each ghosts to the nearest cell on the path decides the next move of the agent. 
       
       For the case of 2 or more moves having same success rate, Agent 4 goes ahead with the 
       goal oriented behaviour(moving in the direction of the goal). 

       If there is no path to the end goal. Agent like agent 2 and agent 3 will move away from the 
       nearest visible ghost, similar to Agent.
       Default start_cell = 0, top left corner.'''

    agent_loc = 0
    agent_move_away = 0
    end_goal = total_cell_n*total_cell_n-1
    while(agent_loc != end_goal):
      print("Agent 4 location: ", agent_loc)
      if agent_loc in ghost_locs:
        print("Agent 4 reached the ghost and is dead")
        return False, agent_move_away
      self_neigh = neighbor_data[agent_loc]
      self_neigh.append(agent_loc)
      success_rate = {}
      path = a_star_search_algo(agent_loc, end_goal, neighbor_data)
      if not path:
        print("All paths are blocked, Agent 4 moving away from nearest visible Ghost")
        maxm = 0
        agent_move_away = agent_move_away + 1
        farthest_neigh = None
        nearest_ghost = get_nearest_visible_ghost(maze, agent_loc, ghost_locs, neighbor_data)
        for neigh in neighbor_data[agent_loc]:
          ghost_agent_path = a_star_search_algo(neigh, nearest_ghost, neighbor_data)
          print("Ghost agent path is ", ghost_agent_path)
          if len(ghost_agent_path) > maxm:
            maxm = len(ghost_agent_path)
            farthest_neigh = neigh
            print("Farthest neighbor is ", farthest_neigh)
            agent_loc = farthest_neigh
        print("Agent 4 moved away from nearest possible ghost. It is now at ", farthest_neigh)
        ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)

      for move in self_neigh:
        if move in ghost_locs:
          success_rate[move] = 0
          continue
        agent_success = 0
        path = a_star_search_algo(move, end_goal, neighbor_data)
        for simulation in range(simulations):
          agent_2_dead = self.agent_2(maze, path, neighbor_data, ghost_locs, agent_loc, agent_simulated=True)
          if agent_2_dead: 
            agent_success = agent_success + 1
        success_rate[move] = (agent_success+choose_survival_oriented_choice(path,ghost_locs))/simulations
      print(" success_rate.items() ",success_rate.items())
      success_rate = {k: v for k, v in sorted(success_rate.items(), key=lambda item: item[1], reverse=True)}
      print("Success rates are ", success_rate)
      if list(success_rate.values())[0] == 0:
        print("There is no probability to win from any perspective")
        if agent_loc == end_goal:
          print("Agent 4 reached the goal")
          return True, agent_move_away
        ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)
        continue
      agent_choices = []
      for key, value in success_rate.items():
        if value == list(success_rate.values())[0]:
          agent_choices.append(key)
      if(len(agent_choices) == 1):
        agent_choice = agent_choices[0]
      else:
        agent_choice = choose_goal_oriented_choice(agent_choices, neighbor_data)
      print("Agent 4 choice is ", agent_choice)
      agent_loc = agent_choice
      if agent_loc in ghost_locs:
        return False, agent_move_away
      if agent_loc == end_goal:
        print("Agent 4 reached the goal")
        return True, agent_move_away
      ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)


def agent_5(self, maze, neighbor_data, ghost_locs, start_cell=0, simulations=5):
    '''Agent 5 is similar to agent 4 and simulates agent 2 from each of the possible moves it can take including 
       its current position. Based on the success rates(# of times Agent 2 is alive) added to the 
       avg. distance of each ghosts to the nearest cell on the path decides the next move of the agent. 
       
       For the case of 2 or more moves having same success rate, Agent 5 goes ahead with the 
       goal oriented behaviour(moving in the direction of the goal). 

       Unlike Agent 4 , Agent 5 can't see the ghost if the are in the blocked cells. 
       The "get_visible_ghosts" funcation filters the locations of the ghosts that are visible to Agnet 5, that 
       is list of all the ghosts that are on unblocked cell. 
       Default start_cell = 0, top left corner.'''

    visible_ghosts = get_visible_ghosts(ghost_locs, maze)
    agent_loc =0
    end_goal = total_cell_n*total_cell_n-1
    agent_move_away = 0
    while(agent_loc != end_goal):
      print("Agent 5 location: ", agent_loc)
      if agent_loc in ghost_locs:
        # Agent reached the ghost
        print("Agent 5 reached the ghost and is dead")
        return False, agent_move_away
      self_neigh = neighbor_data[agent_loc]
      self_neigh.append(agent_loc)
      success_rate = {}
      path = a_star_search_algo(agent_loc, end_goal, neighbor_data)
      if not path:
        print("All paths are blocked, Agent 5 moving away from nearest visible Ghost")
        agent_move_away = agent_move_away + 1
        maxm = 0
        farthest_neigh = None
        nearest_ghost = get_nearest_visible_ghost(maze, agent_loc, ghost_locs, neighbor_data)
        for neigh in neighbor_data[agent_loc]:
          ghost_agent_path = a_star_search_algo(neigh, nearest_ghost, neighbor_data)
          print("Ghost agent path is ", ghost_agent_path)
          if len(ghost_agent_path) > maxm:
            maxm = len(ghost_agent_path)
            farthest_neigh = neigh
            print("Farthest neighbor is ", farthest_neigh)
            agent_loc = farthest_neigh
        print("Agent 5 moved away from nearest possible ghost. It is now at ", farthest_neigh)
        ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)

      for move in self_neigh:
        if move in ghost_locs:
          success_rate[move] = 0
          continue
        agent_success = 0
        path = a_star_search_algo(move, end_goal, neighbor_data)
        for simulation in range(simulations):
          agent_2_dead = self.agent_2(maze, path, neighbor_data, ghost_locs, agent_loc, agent_simulated=True)
          if agent_2_dead: 
            agent_success = agent_success + 1
        visible_ghosts = get_visible_ghosts(ghost_locs, maze)
        success_rate[move] = (agent_success+choose_survival_oriented_choice(path,visible_ghosts))/simulations
      print(" success_rate.items() ",success_rate.items())
      success_rate = {k: v for k, v in sorted(success_rate.items(), key=lambda item: item[1], reverse=True)}
      print("Success rates are ", success_rate)
      if list(success_rate.values())[0] == 0:
        print("There is no probability to win from any perspective")
        if agent_loc == end_goal:
          print("Agent 5 reached the goal")
          return True, agent_move_away
        ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)
        continue
      agent_choices = []
      for key, value in success_rate.items():
        if value == list(success_rate.values())[0]:
          agent_choices.append(key)
      if(len(agent_choices) == 1):
        agent_choice = agent_choices[0]
      else:
        agent_choice = choose_goal_oriented_choice(agent_choices, neighbor_data)
      print("Agent 5 choice is ", agent_choice)
      agent_loc = agent_choice
      if agent_loc == end_goal:
        print("Agent 5 reached the goal")
        return True, agent_move_away
      ghost_locs = self.manage_ghost_location(ghost_locs, neighbor_data)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--agent_1', action='store_true', default=False)
  parser.add_argument('--agent_2', action='store_true', default=False)
  parser.add_argument('--agent_3', action='store_true', default=False)
  parser.add_argument('--agent_4', action='store_true', default=False)
  parser.add_argument('--agent_5', action='store_true', default=False)
  args = parser.parse_args()
  env = The_Environment()
  unique, counts = np.unique(env.maze, return_counts=True)
  print("Env ghosts ", env.ghost_locs)
  print(env.maze)
  if is_ghost_born_in_top_left_cell(env.ghost_locs):
    print("All agents will die")
  agent = Agent()
  if args.agent_1:
    agent.agent_1(env.maze, env.path, env.neighbor_data, env.ghost_locs)
  if args.agent_2:
    agent.agent_2(env.maze, env.path, env.neighbor_data, env.ghost_locs)
  if args.agent_3:
    agent.agent_3(env.maze, env.neighbor_data, env.ghost_locs)
  if args.agent_4:
    agent.agent_4(env.maze, env.neighbor_data, env.ghost_locs)
  if args.agent_5:
    agent.agent_5(env.maze, env.neighbor_data, env.ghost_locs)

if __name__ == '__main__':
  main()