import abc
import math
import numpy as np
from collections import deque
import matplotlib.pyplot as plt

def main():
    step = 0
   # while step < num_iteration:


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

class Grid:
    def __init__(self, x_range, y_range, initialValue = 0):
        self.x = np.arange(x_range[0], x_range[1], x_range[2])
        self.y = np.arange(y_range[0], y_range[1], y_range[2])
        #self.data = [deque([initialValue for i in range(y_range[0], y_range[1] + y_range[2], y_range[2])]) for j in range(x_range[0], x_range[1] + x_range[2], x_range[2])]


    def dist(self, intruder_pos):
        xx, yy = np.meshgrid(self.x, self.y)
        dist_map = (xx - intruder_pos[0])**2 + (yy - intruder_pos[1])**2
        return dist_map

    def get_vertex_neighbours(pos, head, action_space): #ToDo3: define action space
        neighbour = {}
        new_head = head + action_space
        dx = np.cos(new_head) * g_speed * time_step
        dy = np.sin(new_head) * g_speed * time_step
        new_pos = list(zip(pos[0] + dx, pos[1] + dy))
        vertex_x = (pos[0] + dx) // 1
        vertex_y = (pos[1] + dy) // 1
        vertex = list(zip(vertex_x, vertex_y))
        neighbour[vertex] = list(zip(new_pos, new_head))
        return neighbour

    def dist2init_trj(pos, init_trj_para):
        pos_x = pos[0]
        pos_y = pos[1]
        a = init_trj_para[0]
        b = init_trj_para[1]
        return abs(a * pos_x - pos_y + b) / math.sqrt(a**2 + 1)

    def dist2init_point(pos, init_pos):
        return math.sqrt(sum((init_pos - pos)**2))

    '''def edge(self):
        ## TODO:
        return edge #list '''

def heuristic(pos, own_name)：
    init_trj_para = init_plan[own_name][0]
    init_pos = init_plan[own_name][3]
    h1 = world.dist2init_trj(pos, init_trj_para) * A
    pos_x = pos[0]
    pos_y = pos[1]
    a = init_trj_para[0]
    b = init_trj_para[1] # y = a * x + b
    try:
        if pos_y != a * pos_x + b:
            raise ValueError("The init_point is not in the flight plan")
    except ValueError as ve:
        print(ve)
    x = (pos_x + a * pos_y - a * b) / (a**2 + 1)
    y = ((a**2) * pos_y + a * pos_x + b) / (a**2 + 1)
    h2 = math.sqrt((x - init_pos[0])**2 + (y - init_pos[1])**2) * B # ToDo: define A and B
    return h1 + h2

def cst_map(intruder_pos): #for plot figure
    world = Grid(x_range, y_range)
    dist2 = world.dist(intruder_pos)
    cst_map = np.exp(-dist2)
    return cst_map

def st_cst(intruder_pos, own_pos): #for calculation
    dist2 = np.sum((intruder_pos - own_pos)**2)
    st_cst = np.exp(-dist2)
    return st_cst

def plan2pos(init_pos, plan):
    plan_pos = init_pos.copy()
    head_angl = plan * np.transpose(np.tri(len(plan), len(plan), 0))
    delta_x = np.cos(head_angl) * g_speed * time_step
    delta_y = np.sin(head_angl) * g_speed * time_step
    pos_x = delta_x * np.transpose(np.tri(len(delta_x), len(delta_x), 0))
    pos_y = delta_y * np.transpose(np.tri(len(delta_y), len(delta_y), 0))
    pos = np.array(list(zip(pos_x, pos_y))) + init_pos
    return pos

def AStarSearch(intruder_plan, own_name):
    G = {} #Actual movement cost to each position from the start position
    F = {} #Estimated movement cost of start to end going via this position
    init_pos_intruder = []
    for name in init_plan:
        #get the position plan of the intruder(s)
        if name == own_name:
            init_pos = init_plan[name][2]
            init_head = init_plan[name][3]
            start = (init_pos[0]//1, init_pos[1]//1)
            head = init_head
        init_pos_intruder.append(init_plan[name][0])
    intruder_plan2pos = plan2pos(init_pos_intruder, intruder_plan)

    time_step = 0

    # Initialize starting values
    G[start] = [0, init_head]
    F[start] = heuristic(start, own_name) # ToDo 1: heuristic function

    closedVertices = set()
    openVertices = set([start])
    cameFrom = {}
    realPos = {}

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
                time_step += 1

        if currentVertex[0] > x_range[1]:
            #retrace our route backward
            path = [currentVertex]
            while currentVertex in cameFrom:
                currentVertex = cameFrom[currentVertex]
                path.append(currentVertex)
            path.reserve()
            return path, F[currentVertex] #Done!

        #Mark the current vertex as closed
        openVertices.remove(currentVertex)
        closedVertices.add(currentVertex)

        #Update scores for vertices near the current position
        neighbour = get_vertex_neighbours(currentPos, currentHead, action_space)#dict(neighbourVertex: [neighbourPos,neighbourHead])
        for neighbourVertex in neighbour: #ToDo2: get_vertex_neighbours function
            if neighbourVertex in closedVertices:
                continue
            intruder_pos = intruder_plan2pos[time_step]
            neighbourPos = neighbour[neighbourVertex][0]
            candidateG = st_cst(intruder_pos, neighbourPos)
            if neighbourVertex not in openVertices:
                openVertices[neighbourVertex] = neighbour[neighbourVertex] #Discovered a new vertex
            elif candidateG >= G[neighbour]:
                continue

            # Adopt this G score
            cameFrom[neighbourVertex] = currentVertex
            G[neighbourVertex] = candidateG
            H = heuristic(neighbourPos, own_name)
            F[neighbourVertex] = G[neighbourVertex] + H
    raise RuntimeError("A* failed to find a solution")


x_range = np.array([0, 200, 1])   #1 indicates 0.01 nautical mile
y_range = np.array([-100, 100, 1]) #1 indicates 0.01 nautical mile
time_step = 1 #seconds
g_speed = 10 #1 indicates 0.01 nautical mile per second
action_space = [-10*math.pi/180, 0] #heading angle in rad. Nagative sign indicates right hand of the aircraft.
action_cost  = [0.1, 0]
word = Grid(x_range, y_range)
edge = world.edge()
num_iteration = 10
init_plan = {'ac1': [[0,0],[x_range[0] - 100,0], [0,0], 0], 'ac2': [[0,0],[x_range[1] + 100, 0], [x_range,0], math.pi]} #key: name of aircraft, value: [[a,b], a far point in y=ax+b, [init_pos], init_head]
A, B = 1, 1
#Todo: init_trj_para, init_pos
#input: init_plan_ac1

if __name__ == "__main__":
    ac1 = Aircraft("ac1", 0, 0, 0, 0)
    dic_ac1 = ac1.set_ac()
    print(dic_ac1)
    '''z = cst_map([20,0])
    plt.contourf(z)
    plt.show()
    print(0)'''
