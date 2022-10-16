import argparse
import os
import sys
import pandas as pd
import numpy as np
sys.path.append(os.path.abspath(__file__))

from environment import The_Environment
from agent import Agent

def agent_performace(agent_file, no_simulations=100):
  row = 0
  df = pd.DataFrame(columns=['Ghost no', 'Agent 1', 'Agent 2','Agent 3','Agent 4',])
  df.to_csv(agent_file)
  for ghost_num in np.arange(10,130,10):
    df = pd.read_csv(agent_file, index_col=0)
    success_agent1,success_agent2,success_agent3,success_agent4 = 0,0,0,0
    replans_agent2, moves_away_agent2 = [0,0],[0,0]
    for i in range(0,no_simulations):
      env = The_Environment(ghost_count=ghost_num)
      print(" ----------------------- START -----------------------------")
      agent1=Agent().agent_1(env.maze, env.path, env.neighbor_data, env.ghost_locs)
      if agent1 :
        success_agent1 = success_agent1+1
      agent2, replans, agent_move_away = Agent().agent_2_optimised(env.maze, env.path, env.neighbor_data, env.ghost_locs)
      if agent2 :
        success_agent2 = success_agent2+1
        replans_agent2[0] = replans_agent2[0]+replans
        moves_away_agent2[0]=moves_away_agent2[0]+agent_move_away
      else : 
        replans_agent2[1] = replans_agent2[1]+replans
        moves_away_agent2[1]=moves_away_agent2[1]+agent_move_away
      agent3, agent_move_away = Agent().agent_3(env.maze, env.neighbor_data, env.ghost_locs)
      if agent3 :
        success_agent3 = success_agent3+1
      agent4, _ = Agent().agent_4(env.maze, env.neighbor_data, env.ghost_locs)
      if agent4 :
        success_agent4 = success_agent4+1
      print(" ----------------------- END -----------------------------")
    df.loc[row,['Agent 1']]= success_agent1
    df.loc[row, ["Agent 2"]] = success_agent2
    df.loc[row, ["Agent 3"]] = success_agent3
    df.loc[row, ["Agent 4"]] = success_agent3
    df.loc[row, ["Ghost no"]] = ghost_num
    row = row + 1
    print(replans_agent2, moves_away_agent2)
    df.to_csv(agent_file)
      
    print(" agent 1 live --  -------->",success_agent1," for ghost no---> ",ghost_num)
    print(" agent 2 live --  -------->",success_agent2," for ghost no---> ",ghost_num)
    print(" agent 3 live --  -------->",success_agent3," for ghost no---> ",ghost_num)
    print(" agent 4 live --  -------->",success_agent4," for ghost no---> ",ghost_num)
  print(df)

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--data-file', default='agent_data.csv')
  args = parser.parse_args()
  agent_performace(args.data_file)

if __name__ == '__main__':
  main()