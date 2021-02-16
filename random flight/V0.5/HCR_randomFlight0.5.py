import abc
import math
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import random

def generate_random_flight(number_of_flight):
    ac = {}
    for i in range(number_of_flight):
        while True:
            name = 'ac{}'.format(i)
            init_pos_x = random.randint(x_range[0], x_range[1])
            init_pos_y = random.randint(y_range[0], y_range[1])
            init_head = random.randint(0, 360) * math.pi / 180
            a = np.tan(init_head)
            b = init_pos_y - a * init_pos_x
            flight_range = (x_range[1] - x_range[0])/2
            waypoint_x = init_pos_x + flight_range * np.cos(init_head)
            waypoint_y = init_pos_y + flight_range * np.sin(init_head)
            if waypoint_x > x_range[1] or waypoint_x < x_range[0] or waypoint_y > y_range[1] or waypoint_y < y_range[0]:
                continue
            ac[name] = [[a,b], [waypoint_x, waypoint_y], [init_pos_x, init_pos_y], init_head]
            break
    return ac


class Grid:
    def __init__(self, x_range, y_range, initialValue = 0):
        self.x = np.arange(x_range[0], x_range[1], x_range[2])
        self.y = np.arange(y_range[0], y_range[1], y_range[2])
        #self.data = [deque([initialValue for i in range(y_range[0], y_range[1] + y_range[2], y_range[2])]) for j in range(x_range[0], x_range[1] + x_range[2], x_range[2])]

    def dist(self, intruder_pos):
        xx, yy = np.meshgrid(self.x, self.y)
        dist_map = np.sqrt((xx - intruder_pos[0])**2 + (yy - intruder_pos[1])**2)
        return dist_map

    def get_vertex_neighbours(self, pos, head, action_space, time): #ToDo3: define action space
        neighbour = {}
        new_head = list(head + action_space)
        dx = np.cos(new_head) * g_speed * time_step
        dy = np.sin(new_head) * g_speed * time_step
        new_pos = list(zip(pos[0] + dx, pos[1] + dy))
        vertex_x = (pos[0] + dx) // 1
        vertex_y = (pos[1] + dy) // 1
        new_time = (time + 1) * np.ones(len(dx))
        vertex = list(zip(vertex_x, vertex_y))
        poshead = list(zip(new_pos, new_head, new_time))
        jj = 0
        for ii in vertex:
            neighbour[ii] = poshead[jj]
            jj += 1
        #neighbour[vertex] = list(zip(new_pos, new_head))
        return neighbour

    def dist2init_trj(self, pos, init_trj_para):
        pos_x = pos[0]
        pos_y = pos[1]
        a = init_trj_para[0]
        b = init_trj_para[1]
        return abs(a * pos_x - pos_y + b) / math.sqrt(a**2 + 1)

    def point_dist(self, pos, init_pos):
        pos = np.array(pos)
        init_pos = np.array(init_pos)
        return math.sqrt(sum((init_pos - pos)**2))

class Aircraft:
    def __init__(self, name, x_pos, y_pos, heading, plan):
        self.name = name
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.heading = heading
        self.plan = plan # list of action (delta_angle)

    def set_ac(self): #using dictionary to describe an aircraft
        ac_dic = dict()
        ac_dic[self.name] = [self.x_pos, self.y_pos, self.heading, self.plan]
        return ac_dic

    def init_plan_traj(self, a,b,x):
        y = a * x + b
        return [x, y]

def heuristic(pos, own_name):
    init_trj_para = init_plan[own_name][0]
    init_pos = init_plan[own_name][2]
    waypoint = init_plan[own_name][1]
    trvl_dist = world.point_dist(waypoint, init_pos) #distance from start point to waypoint
    wp_cst = AA * trvl_dist + 1 #AA is a negative parameter
    h = wp_cst - world.point_dist(pos, waypoint) * AA
    a = init_trj_para[0]
    b = init_trj_para[1] # y = a * x + b
    try:
        if init_pos[1] != a * init_pos[0] + b or waypoint[1] != a * waypoint[0] + b:
            raise ValueError("The start point or waypoint is not in the flight plan")
    except ValueError as ve:
        print(ve)
    return h

def chebishev_heuristic(start, goal):
    # Use Chebyshev distance heuristic if we can move one square either
    # adjacent or diagonal
    D = 0.1
    D2 = 0.1
    dx = abs(start[0] - goal[0])
    dy = abs(start[1] - goal[1])
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)


def heuristic_cos(pos, head, own_name):
    waypoint = np.array(init_plan[own_name][1])
    pos = np.array(pos)
    vector = waypoint - pos
    ang = math.atan2(vector[1], vector[0])
    '''if vector[0] >= 0:
        ang = math.atan2(vector[1], vector[0])
    else:
        ang = math.atan2(vector[1], vector[0]) + math.pi'''
    theta = head - ang
    h = (5 - math.cos(theta)) * CC
    return h


def heuristic_dis(current_pos, neighbour_pos,own_name):
    waypoint = init_plan[own_name][1]
    r = world.point_dist(neighbour_pos, waypoint)
    R = world.point_dist(current_pos, waypoint)
    rmax = g_speed * time_step
    h = (1 - (R-r)/rmax) * DD
    return h

def heuristic_fp(current_pos, neighbour_pos,own_name):
    init_trj_para = init_plan[own_name][0]
    h1 = world.dist2init_trj(neighbour_pos, init_trj_para) * EE

    waypoint = init_plan[own_name][1]
    r = world.point_dist(neighbour_pos, waypoint)
    R = world.point_dist(current_pos, waypoint)
    rmax = g_speed * time_step
    h2 = (1 - (R - r) / rmax) * DD
    return h1+h2

def cst_map(intruder_pos): #for plot figure
    world = Grid(x_range, y_range)
    dist2 = world.dist(intruder_pos)
    cst_map = np.exp(-dist2)
    return cst_map

def st_cst(intruder_pos, own_pos): #for calculation
    own_pos_temp = np.array(own_pos)
    dist2 = np.sum((intruder_pos - own_pos_temp)**2, 1)
    state_cost = np.sum(np.exp(-dist2/2500))
    return state_cost

def head_cst(current_head, init_head):
    return ((current_head - init_head) * 180 / (10 * math.pi)) * CC

def plan2pos(init_pos, init_head, plan):
    start_pos = init_pos.copy()
    plan = np.array(plan)
    head_angl = plan.dot(np.transpose(np.tri(len(plan), len(plan), 0))) + init_head
    delta_x = np.cos(head_angl) * g_speed * time_step
    delta_y = np.sin(head_angl) * g_speed * time_step
    pos_x = delta_x.dot(np.transpose(np.tri(len(delta_x), len(delta_x), 0))) + start_pos[0]
    pos_y = delta_y.dot(np.transpose(np.tri(len(delta_y), len(delta_y), 0))) + start_pos[1]
    pos = np.array(list(zip(pos_x, pos_y)))
    return pos

def AStarSearch(intruder, own_name):
    G = {} #Actual movement cost to each position from the start position
    F = {} #Estimated movement cost of start to end going via this position

    #get the position plan of the intruder(s)
    init_pos = init_plan[own_name][2]
    init_head = init_plan[own_name][3]
    start = (init_pos[0]//1, init_pos[1]//1)
    end_pos = init_plan[own_name][1]
    end = (end_pos[0]//1, end_pos[1]//1)

    # Initialize starting values
    G[start] = [0, init_head]
    F[start] = heuristic_cos(start, init_head, own_name) # ToDo 1: heuristic function

    closedVertices = set()
    openVertices = {start: [init_pos, init_head, 0]}
    cameFrom = {}
    currentTime = 0

    while len(openVertices) > 0:
        #Get the vertex in the open list with the lowest F score
        currentVertex = None
        currentFscore = None
        for vertex in openVertices:
            if currentVertex is None or F[vertex] < currentFscore:
                currentFscore = F[vertex]
                currentVertex = vertex
                currentHead = openVertices[vertex][1]
                currentPos = openVertices[vertex][0]
                currentTime = openVertices[vertex][2]
        intruder_pos = []
        for name in intruder:
            if currentTime >= np.shape(intruder[name])[0]-1:
                intruder_pos.append(intruder[name][-1])
            else:
                intruder_pos.append(intruder[name][int(currentTime)+1])

        #if currentVertex[0] > x_range[1] or currentVertex[0] < x_range[0]:
        dis2end = world.point_dist(currentVertex, end_pos)
        if dis2end <= g_speed * time_step:
            #retrace our route backward
            path = [currentVertex]
            act = [currentHead * 180 / math.pi]
            while currentVertex in cameFrom:
                currentAct = cameFrom[currentVertex][1]
                currentVertex = cameFrom[currentVertex][0]
                path.append(currentVertex)
                act.append(currentAct * 180 / math.pi)
            act = list((np.array(act) + 0.01) // 1)
            path.reverse()
            act.reverse()
            return path, act #Done!

        #Mark the current vertex as closed.
        del(openVertices[currentVertex])
        closedVertices.add(currentVertex)

        #Update scores for vertices near the current position
        neighbour = world.get_vertex_neighbours(currentPos, currentHead, action_space, currentTime)#dict(neighbourVertex: [neighbourPos,neighbourHead])
        for neighbourVertex in neighbour: #ToDo2: get_vertex_neighbours function
            if neighbourVertex in closedVertices:
                continue
            neighbourPos = neighbour[neighbourVertex][0]
            candidateG = st_cst(intruder_pos, neighbourPos)
            if neighbourVertex not in openVertices:
                openVertices[neighbourVertex] = neighbour[neighbourVertex] #Discovered a new vertex
            elif candidateG >= G[neighbourVertex]:
                continue

            # Adopt this G score
            neighbourHead = neighbour[neighbourVertex][1]
            delta_angle = neighbourHead - currentHead
            cameFrom[neighbourVertex] = [currentVertex, delta_angle]
            G[neighbourVertex] = candidateG
            H = heuristic_cos(currentPos, neighbourHead, own_name)
            #H2 = chebishev_heuristic(neighbourPos, init_plan[own_name][1])
            #headCost = head_cst(neighbour[neighbourVertex][1], init_head)
            #actionCost = abs(delta_angle) * DD
            F[neighbourVertex] = G[neighbourVertex] + H
    raise RuntimeError("A* failed to find a solution")


def main():
    iter_step = 0
    path = {}

    for ac in init_plan:
        x = init_plan[ac][2][0] + math.cos(init_plan[ac][3]) * g_speed * np.arange(0, maxTimeStep)
        y = init_plan[ac][2][1] + math.sin(init_plan[ac][3]) * g_speed * np.arange(0, maxTimeStep)
        path[ac] = np.array(list(zip(x, y)))
        plt.plot(*zip(*path[ac]), '--k')
        plt.ylim(y_range[0], y_range[1])
        plt.xlim(x_range[0], x_range[1])
    while iter_step < num_iteration:
        for ac in order:
            intruder_path = path.copy()
            del intruder_path[ac]
            path[ac], act = AStarSearch(intruder_path, ac)
            plt.plot(*zip(*path[ac]), '-k.')
        plt.ylim(y_range[0], y_range[1])
        plt.xlim(x_range[0], x_range[1])
        plt.savefig('{}iteration'.format(iter_step))
        plt.close()
        iter_step += 1

    '''plt.plot(*zip(*path['ac0']), '-k.', *zip(*path['ac1']), '-r.', *zip(*path['ac2']), '-b.', *zip(*path['ac3']), '-g.',[x_range[0], x_range[1]],[y_range[0], y_range[1]],'--y', [x_range[0], x_range[1]],[y_range[1], y_range[0]],'--y')
    plt.ylim(y_range[0], y_range[1])
    plt.xlim(x_range[0], x_range[1])
    plt.show()'''

x_range = np.array([-500, 500, 1])  # 1 indicates 0.01 nautical mile
y_range = np.array([-500, 500, 1])  # 1 indicates 0.01 nautical mile
time_step = 1  # seconds
g_speed = 20  # 1 indicates 0.01 nautical mile per second
action_space = np.array([-5 * math.pi / 180, 0, 5 * math.pi / 180])  # heading angle in rad. Nagative sign indicates right hand of the aircraft.
action_cost = [0.1, 0]
init_plan = generate_random_flight(10)
AA = -0.01
CC = 5
DD = 0.1
EE = 0.001
world = Grid(x_range, y_range)
order = list(init_plan.keys())
num_iteration = 20
maxTimeStep = 70


if __name__ == "__main__":
    main()