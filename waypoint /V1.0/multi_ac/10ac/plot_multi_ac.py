import math
import numpy as np
import matplotlib.pyplot as plt


def init_flight_plan(num_ac, radial):
    a_deg = 360/num_ac
    a_rad = a_deg * math.pi/180
    init_plan = {}
    for i in np.arange(0, num_ac):
        theta = i * a_rad
        x = -radial*np.cos(theta)
        y = -radial*np.sin(theta)
        way_point_x = -x
        way_point_y = -y
        init_plan.update({'ac{}'.format(i+1): [[0], (way_point_x, way_point_y), (x,y), theta]})
    return init_plan

init_plan = init_flight_plan(10, 500)
fig, ax = plt.subplots()
for i in np.arange(0,5):
    x1 = init_plan['ac{}'.format(i+1)][1][0]
    y1 = init_plan['ac{}'.format(i+1)][1][1]
    x2 = init_plan['ac{}'.format(i+1)][2][0]
    y2 = init_plan['ac{}'.format(i+1)][2][1]
    ax.plot([x1,x2], [y1,y2], '--.k')
for i in np.arange(0,10):
    circle = plt.Circle(init_plan['ac{}'.format(i + 1)][1], 25, color='k', fill=False, ls='--')
    ax.add_artist(circle)
plt.axis('equal')
xstart, xend = ax.get_xlim()
xticks = np.arange(xstart, xend + 1, 100)
ax.set_xticks(xticks)
ax.set_xticklabels((xticks / 5).astype(int))
ystart, yend = ax.get_ylim()
yticks = np.linspace(ystart, yend, 3).astype(int)
ax.set_yticks(yticks)
ax.set_yticklabels(yticks / 5)
ax.set_xlabel('nautical mile')
ax.set_ylabel('nautical mile')
plt.show()