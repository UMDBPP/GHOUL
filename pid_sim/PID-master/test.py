import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import csv

time = []
ascent_rate = []
vent_cmds = []
altitude = []
filtered_ascent_rate = []

with open('pid_output.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter = ',')
    next(plots)
    next(plots)


    for row in plots:
        time.append(int(row[0]))
        altitude.append(int(row[1]) / 1000)
        vent_cmds.append(int(row[2]))
        ascent_rate.append(int(row[3]) / 1000)
        filtered_ascent_rate.append(int(row[4]) / 1000)




fig, ax1 = plt.subplots()

color = 'tab:blue'
# ax1.get_yaxis().set_ticks([])
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Vent Open', color=color)
ax1.plot(time, vent_cmds, color=color, alpha=0.2)

ax2 = ax1.twinx()  

color = 'tab:red'
ax2.set_ylabel('Altitude (m)', color=color)
ax2.plot(time, altitude, color=color)
ax2.ticklabel_format(useOffset=False, style='plain')

ax3 = ax1.twinx() 

ax3.spines.right.set_position(("axes", 1.2))

color = 'tab:green'
ax3.set_ylabel('Ascent Rate (m/s)', color=color)
ax3.plot(time, ascent_rate, color=color, alpha=0.2)
ax3.plot(time, filtered_ascent_rate, color=color, alpha=0.4)


fig.tight_layout()
plt.show()

