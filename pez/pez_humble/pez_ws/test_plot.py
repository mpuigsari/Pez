#!/usr/bin/env python3
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import time

plt.ion()                       # <-- interactive mode on
fig, ax = plt.subplots()
ax.plot([0,1,2,3],[1,3,2,4])
plt.show()                     # <-- force window now

time.sleep(2)                  # keep it up for 2 seconds
