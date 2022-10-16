import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import pandas as pd

df = pd.read_csv('agent_data.csv') 

labels=['Agent 1','Agent 2','Agent 3','Agent 4']
fig, ax = plt.subplots(figsize=(15, 10))

ax.plot(df['Ghost no'], df.loc[:,('Agent 1')], label='Agent 1')  # Plot some data on the axes.
ax.plot(df['Ghost no'], df.loc[:,('Agent 2')], label='Agent 2')  # Plot more data on the axes...
ax.plot(df['Ghost no'], df.loc[:,('Agent 3')], label='Agent 3')  # ... and some more.
ax.plot(df['Ghost no'], df.loc[:,('Agent 4')], label='Agent 4')  # ... and some more.
ax.legend(labels)
ax.set_xlabel('No of Ghosts')  
ax.set_ylabel('none ')