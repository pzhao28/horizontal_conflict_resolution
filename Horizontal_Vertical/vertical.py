import abc
import math
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from IPython.display import HTML
import scipy.io as sio
import sys
import copy
import pylab as pl

def main():
    step = 0
    total_cost = np.zeros(num_iteration, dtype=np.float)
    world = Grid([0, interval, 1], height_range)

    hspace = .4
    pl.subplots_adjust(hspace=hspace)
    fig, axs = plt.subplots(2, num_ac)

    while step < num_iteration:
        print("\riteration {}/{}".format(step, num_iteration-1), end="")
        sys.stdout.flush()
        #TODO0: define set_order()
        all_cost_dic = dict()
        beta = 360
        '''if step == 0:
            order = init_sort()'''
        for key in order:
            #TODO1: define slv_mdp
            cst_map = world.cost_map(key, order, all_ac, T_to_hit, beta)
            #else:
                #cst_map = world.cost_map(key, order, all_ac, T_to_hit, beta)
            # sio.savemat("cst_map.mat", {'cst_map': cst_map})
            (opt_action, opt_trajectory, min_cost) = slv_mdp(start_state, cst_map)
            trvl_plan = opt_trajectory[0]
            all_ac[key][2] = trvl_plan[0:interval] #update travel travel plan
            all_cost_dic[key] = min_cost
            if step == num_iteration - 1:
                #plt_cst_map(axs[i, 0], cst_map)
                axs[0].imshow(np.transpose(cst_map), origin='lower', extent=[0, 200, -2000, 2000], aspect="auto")
                #axs[i, 0].set_yticks(np.arange(-1000, 1000, 100))
                #plt.colorbar(subfig, cax = axs[i,0])
                axs[0].set_title('Cost Map of Aircraft A')
                axs[0].set_xlabel('Time(s)')
                axs[0].set_ylabel('Relative height(ft)')
                #plt_trvl_plan(axs[i, 1], trvl_plan)
                axs[1].plot((trvl_plan - start_state[0]) * height_range[2]*50/6, label = 'Proposed method')
                axs[1].plot([0,200],[0,0], 'r--', label='Utility fusion strategy')
                axs[1].set_title('Path Planning of Aircraft A')
                axs[1].set_xlabel('Time(s)')
                axs[1].set_ylabel('Relative height(ft)')
                axs[1].set_ylim(-2000, 2000)
                axs[1].legend()

        total_cost[step] = np.sum(list(all_cost_dic.values())).astype(float)

        #order.reverse()
        step += 1
    plt.tight_layout()
    plt.savefig('Final Results')
    plt.legend()
    plt.close()
    sio.savemat("total_cost", {'total_cost': total_cost})
    #animation = ap.anim_plt(all_ac['ac1'][2], all_ac['ac2'][2], all_ac['ac3'][2])
    #animation.save('animation_av.gif', writer='imagemagick', fps=20)



class Aircraft:
    def __init__(self, height, velocity, trvl_plan):
        self.height = height
        self.velocity = velocity

    def set_ac(self, name, height, velocity, trvl_plan):
        self.name = name
        self.height = height
        self.velocity = velocity
        self.trvl_plan = trvl_plan
        ac_dic = dict()
        ac_dic[name] = [height, velocity, trvl_plan]
        return ac_dic

    def height_update(self, action):
        self.volocity = action
        self.height += action * 1
        return self.height


class Grid:
    '''
    2D discretized grid of the world
    '''
    def __init__(self, x_range, y_range, initialValue = 0):
        self.x_range = x_range #list[x_lower, x_upper, x_scale]
        self.y_range = y_range #list[y_lower, y_upper, y_scale]
        self.x = np.arange(self.x_range[0], self.x_range[1], self.x_range[2])
        self.y = np.arange(self.y_range[0], self.y_range[1] + self.y_range[2], self.y_range[2])
        self.data = [deque([initialValue for i in range(self.y_range[0], self.y_range[1] + self.y_range[2], self.y_range[2])]) for j in range(self.x_range[0], self.x_range[1] + self.x_range[2], self.x_range[2])]
        #self.data = [deque([initialValue for x in range(x_range)]) for y in range(y_range)]

    def copy(self):
        copy = Grid(self.x_range, self.y_range)
        for i in range(self.x_range):
            for j in range(self.y_range):
                copy.data[j][i] = self.data[j][i]
        return copy

    def discrete_h(self, height):
        if height < self.y_range[0]:
            disc_h = self.y_range[0]
            idx_h = 0
        elif height >= self.y_range[1]:
            disc_h = self.y_range[1]
            idx_h = (self.y_range[1]-self.y_range[0])//self.y_range[2]
        else:
            disc_h = height // self.y_range[2] * self.y_range[2]
            idx_h = (height - self.y_range[0]) // self.y_range[2]
        return [idx_h, disc_h]

    def idx_to_h(self, idx):
        h_list = []
        for i in idx:
            h_list.append(self.y[i])
        return h_list

    def state_map(self):
        st_data = np.array(self.data)
        for i in range(len(self.x)):
            st_data[i] = np.array(self.y)
        return st_data

    def cost_map(self, own_name, order, all_ac, T_to_hit, beta):
        cst_data = np.array(self.data, dtype = np.float64)
        for i in range(len(self.x)):
            for j in range(len(self.y)):
                cst_data[i][j] = state_cost(own_name, order, all_ac, T_to_hit, i, self.y[j], beta)
        return cst_data

    def discrete_v(self, velocity):
        if velocity < velocity_range[0]:
            disc_v = velocity_range[0]
            idx_v = 0
        elif velocity >= velocity_range[1]:
            disc_v = velocity_range[1]
            idx_v = (velocity - velocity_range[0]) // velocity_range[2]
        else:
            disc_v = velocity // velocity_range[2] * velocity_range[2]
            idx_v = (velocity - velocity_range[0]) // velocity_range[2]
        return [idx_v, disc_v]

    def trans_func(self, idx_h, idx_v, idx_a):
        disc_velocity = (idx_v - 0.5 * (velocity_range[1] - velocity_range[0]) / velocity_range[2]) * velocity_range[2]
        next_velocity = action[idx_a]
        idx_v_to_h = next_velocity * 1 // height_range[2]
        next_height_idx = idx_h + idx_v_to_h
        max_h = self.discrete_h(self.y_range[1])
        if next_height_idx > max_h[0]:
            next_height_idx = max_h[0]
        elif next_height_idx < 0:
            next_height_idx = 0
        [next_velocity_idx, next_velocity_disc] = self.discrete_v(next_velocity)
        next_height_idx, next_velocity_idx = int(next_height_idx), int(next_velocity_idx)
        return next_height_idx, next_velocity_idx


def state_cost(own_name, order, all_ac, T_to_hit, step, own_height, beta):
    intruder = dict()
    intruders_plan = []
    for key in all_ac:
        if key == own_name:
            hit_point = np.array(T_to_hit[key])
            hit_point = hit_point[np.nonzero(hit_point)]
            continue
        else:
            intruder[key] = all_ac[key]
            intruders_plan.append(intruder[key][2])
    intruders_plan = np.transpose(intruders_plan)
    relative_height = np.array(own_height - world.idx_to_h(intruders_plan[step]))
    hit_point = np.array(hit_point)
    time_to_hit = hit_point - step
    st_cost = np.sum(np.exp(-(relative_height*(50/6)/beta)**2) * np.exp(-(time_to_hit/60)**2))
    return st_cost

'''def init_sort():
    init_cost_dic = dict()
    for key in all_ac:
        init_cost = state_cost(key, all_ac, T_to_hit, start_state[2], start_height)
        init_cost_dic[key] = init_cost
    #order = sorted(init_cost_dic)
    order = sorted(init_cost_dic, key=lambda x: init_cost_dic[x], reverse=True)
    return order'''


def slv_mdp(start_state, cost_map):

    # value map: each state in state space (height_j, velovity_j, t+1) has a tuple with size 4.
    # The first entry means the minoimum cost from start_state to this state.
    # The third and fourth entres is the optimal previous state (height, velovity, tt), and its optimal action is stored in the second entry.
    # In an word, this tuple record the path from the start state to the terminal state (some height, some velocity, interval).
    # Reference: Wikipedia: Dijkstra's algorithm

    # Output Args: opt_action, opt_trajectory, min_cost:
    # (1) opt_trajectory: (interval+1 by 2) matrix, opt_trajectory[start_state[2],:] is initial state, opt_trajectory[interval,:] is
    # the terminal state by the end of the interval. There are  interval-start_state[2] time slots in between opt_trajectory[start_state[2]+tau,:]
    # is the state of the aircraft by the end of tau th time slot.
    # (2) opt_action: (interval by 1) vector,  opt_trajectory[start_state[2],:] + action[start_state[2]] = opt_trajectory[start_state[2]+1,:],
    # opt_trajectory[interval-1,:] + action[interval-1] = opt_trajectory[interval,:], terminal state does NOT have action.

    num_height_level = (height_range[1] - height_range[0])//height_range[2] + 1
    num_velocity_level = int((velocity_range[1] - velocity_range[0])//velocity_range[2] + 1)
    value_map = np.zeros((num_height_level, num_velocity_level, interval+1, 4), dtype=np.float64)
    value_map[:, :, start_state[2]+1:, 0] = value_map[:, :, start_state[2]+1:, 0] + np.inf
    value_map[start_state[0], start_state[1], start_state[2], 0] = cost_map[start_state[2], start_state[0]]
    active_map = np.zeros((num_height_level, num_velocity_level), dtype=np.int)
    active_map[start_state[0], start_state[1]] = 1
    for tt in np.arange(start_state[2], interval):
        new_active_map = np.zeros((num_height_level, num_velocity_level))
        (row, col) = np.nonzero(active_map)
        for ii in np.arange(row.size):
            for aa in np.arange(5):
                (height_j, velocity_j) = world.trans_func(row[ii], col[ii], aa)
                if value_map[row[ii], col[ii], tt, 0] + action_cost[aa] + cost_map[tt+1, height_j] < \
                        value_map[height_j, velocity_j, tt + 1, 0]:
                    new_active_map[height_j, velocity_j] = 1
                    value_map[height_j, velocity_j, tt + 1, 0] = value_map[row[ii], col[ii], tt, 0] + action_cost[aa] + cost_map[tt+1, height_j]
                    value_map[height_j, velocity_j, tt + 1, 1] = aa
                    value_map[height_j, velocity_j, tt + 1, 2] = row[ii]
                    value_map[height_j, velocity_j, tt + 1, 3] = col[ii]
        active_map = new_active_map
    min_cost = np.amin(value_map[:, :, interval, 0]) # Optput, minimum accumulate cost
    (min_row, min_col) = np.nonzero(value_map[:, :, interval, 0] == min_cost)
    opt_action = np.zeros(interval, dtype = np.int) # Output: optimal action, entries before start_state[2] is 0.
    opt_trajectory = np.zeros([interval+1, 2], dtype = np.int) # Output: optimal trajectory, (height, velocity), entries before start_state[2] is 0.
    opt_trajectory[interval] = (min_row[0], min_col[0])
    for tt in np.flip(np.arange(start_state[2], interval)):
        opt_action[tt] = value_map[opt_trajectory[tt+1, 0], opt_trajectory[tt+1, 1], tt+1, 1]
        opt_trajectory[tt] = value_map[opt_trajectory[tt+1, 0], opt_trajectory[tt+1, 1], tt+1, 2:]
    opt_trajectory = np.transpose(opt_trajectory)
    return opt_action, opt_trajectory, min_cost


def plt_cst_map(ax, cst_map):
    z = ax.imshow(np.transpose(cst_map),origin='lower',extent=extent, aspect="auto")
    plt.xlabel("time(s)")
    plt.ylabel("relative height(ft)")
#    plt.colorbar(z, cax = ax)

def plt_trvl_plan(ax, trvl_plan):
    ax.plot((trvl_plan - start_state[0]) * height_range[2])
    plt.xlabel("time(s)")
    plt.ylabel("relative height(ft)")
    plt.ylim(height_range[0],height_range[1])

# Initialize
#T_to_hit = [[0,140,150],[140,0,160],[150,160,0]] #case1: three aircrafts hit in 3 points
#T_to_hit = [[0,100,100,100,100],[100,0,100,100,100],[100,100,0,100,100],[100,100,100,0,100],[100,100,100,100,0]] #case2: three aircrafts hit in 1 point
T_to_hit = {'ac1':[0,100,100],'ac2':[100,0,100], 'ac_3':[100,100,0]}
num_ac = len(T_to_hit.keys())
interval = 200
height_range = np.array([-240, 240, 1])
velocity_range = np.array([-5, 5, 0.5])
#action = np.array([-2500, -1500, 0, 1500, 2500])/60
action = np.array([-5, -3, 0, 3, 5])
action_cost = np.array([0.04, 0.02, 0, 0.02, 0.04]) * num_ac/2
start_state = [int((0-height_range[0])//height_range[2]), int((0-velocity_range[0])//velocity_range[2]), 0]
init_plan = np.array([start_state[0]] * interval)
ac = Aircraft(0, 0, init_plan)
ac1 = ac.set_ac('ac1', 0, 0, np.array([0]*interval))
ac2 = ac.set_ac('ac2', 180, 0, np.array([180]*interval))
ac3 = ac.set_ac('ac3', -180, 0, np.array([-180]*interval))
all_ac = {**ac1, **ac2, **ac3}
start_height = start_state[0] * height_range[2] + height_range[0]
world = Grid([0, interval, 1], height_range)
num_iteration = 1

cord_y = np.arange(height_range[0],height_range[1],height_range[2])
cord_x = np.arange(0,interval,1)
extent = np.min(cord_x),np.max(cord_x),np.min(cord_y),np.max(cord_y)

order = ['ac1'] #fixed order! Do not change exept adding or deleting.
num_ac = len(order)


if __name__ == "__main__":
    '''world = Grid([0, interval, 1], height_range)
    st_map = world.state_map()
    cst_map = world.cost_map('ac1', all_ac, T_to_hit)
    (opt_action, opt_trajectory, min_cost) = slv_mdp(start_state, cst_map)
    #print(init_sort())
    #np.savetxt('cost_map.csv', cst_map, delimiter=',')
    #print(opt_action)
    print(opt_trajectory[0])'''
    main()
