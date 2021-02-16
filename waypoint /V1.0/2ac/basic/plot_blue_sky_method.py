import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

fig, ax = plt.subplots(1, 1)
x_range = np.array([-50, 50, 20])  # 1 indicates 0.01 nautical mile
y_range = np.array([-25, 25, 25])  # 1 indicates 0.01 nautical mile
ax.plot([x_range[0], x_range[1]],[0, 0],'--r')



ax.plot(0, 5.1, '>k', label = 'own aircraft')
ax.plot(0, 0, '<r', label = 'intruder')
ax.plot([-50, 0], [0, 5.1], '--k', label = 'path using SSD method')
ax.plot([0, 50], [5.1, 0], '--k')
circle1 = plt.Circle((0, 5), 5, color='k', fill=False, ls='--', label = 'protection zone of aircraft 1')
circle2 = plt.Circle((0, 0), 5, color='r', fill=False, ls='--', label = 'protection zone of aircraft 2')
#ax.add_artist(circle1)
ax.add_artist(circle2)
plt.ylim(y_range[0], y_range[1])
plt.xlim(x_range[0], x_range[1])
ax.set_aspect('equal', 'box')
ax.set_xlabel('nautical mile')
ax.set_ylabel('nautical mile')
ax.legend()
plt.show()

'''circle1 = plt.Circle((arrow_pos_x1, arrow_pos_y1), 25, color='k', fill=False, ls='--')
circle2 = plt.Circle((arrow_pos_x2, arrow_pos_y2), 25, color='r', fill=False, ls='--')
ax.add_artist(circle1)
ax.add_artist(circle2)
ax.set_ylim(y_range[0] / 2, y_range[1] / 2)
ax.set_xlim(x_range[0], x_range[1])'''