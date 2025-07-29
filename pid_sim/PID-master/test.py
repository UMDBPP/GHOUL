import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import csv

time = []
ascent_rate = []
vent_cmds = []

with open('pid_output.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter = ',')
    next(plots)
    next(plots)


    for row in plots:
        time.append(int(row[0]))
        ascent_rate.append(int(row[1]) / 1000)
        vent_cmds.append(int(row[2]))


fig, ax1 = plt.subplots()

color = 'tab:blue'
# ax1.get_yaxis().set_ticks([])
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Vent Open', color=color)
ax1.plot(time, vent_cmds, color=color, alpha=0.2)

ax2 = ax1.twinx()  

color = 'tab:red'
ax2.set_ylabel('Ascent Rate (m/s)', color=color)
ax2.plot(time, ascent_rate, color=color)

fig.tight_layout()
plt.show()

