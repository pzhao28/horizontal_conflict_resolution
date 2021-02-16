import abc
import math
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import numpy.matlib as mat

class Grid:
    def __init__(self, x_range, y_range, initialValue = 0):
        self.x = np.arange(x_range[0], x_range[1], x_range[2])
        self.y = np.arange(y_range[0], y_range[1], y_range[2])
        #self.data = [deque([initialValue for i in range(y_range[0], y_range[1] + y_range[2], y_range[2])]) for j in range(x_range[0], x_range[1] + x_range[2], x_range[2])]

    def dist(self, intruder_pos):
        xx, yy = np.meshgrid(self.x, self.y)
        dist_map = np.sqrt((xx - intruder_pos[0])**2 + (yy - intruder_pos[1])**2)
        return dist_map

    def get_vertex_neighbours(self, pos, head, action_space, time, g_speed): #ToDo3: define action space
        neighbour = {}
        new_head = list(head + action_space)
        new_head_rep = mat.repmat(new_head, len(g_speed),1)
        g_speed_rep = np.transpose(mat.repmat(g_speed, len(new_head), 1))
        dx = np.cos(new_head) * g_speed_rep * time_step
        dy = np.sin(new_head) * g_speed_rep * time_step
        dx = dx.flatten()
        dy = dy.flatten()
        new_pos = list(zip(pos[0] + dx, pos[1] + dy))
        vertex_x = (pos[0] + dx) // 1
        vertex_y = (pos[1] + dy) // 1
        new_time = (time + 1) * np.ones(len(dx))
        vertex = list(zip(vertex_x, vertex_y))
        poshead = list(zip(new_pos, new_head_rep.flatten(), new_time, g_speed_rep.flatten()))
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

def heuristic_cos(pos, head, own_name):
    waypoint = np.array(init_plan[own_name][1])
    pos = np.array(pos)
    vector = waypoint - pos
    ang = math.atan2(vector[1], vector[0])
    theta = head - ang
    h = (5 - math.cos(theta)) * CC
    return h

def chebishev_heuristic(start, goal):
    # Use Chebyshev distance heuristic if we can move one square either
    # adjacent or diagonal
    D = 0.1
    D2 = 0.1
    dx = abs(start[0] - goal[0])
    dy = abs(start[1] - goal[1])
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

def cst_map(intruder_pos): #for plot figure
    world = Grid(x_range, y_range)
    dist2 = world.dist(intruder_pos)
    cst_map = np.exp(-dist2)
    return cst_map

def st_cst(intruder_pos, own_pos): #for calculation
    own_pos_temp = np.array(own_pos)
    dist2 = np.sum((intruder_pos - own_pos_temp)**2, 1)
    dist = np.sqrt(dist2)
    #print(np.isnan(dist[dist < SEP]).any())
    if len(dist[dist < SEP]) == 0:
        state_cost = np.sum(np.exp(-(dist-SEP)**2 / gamma2))
    else:
        state_cost = sys.maxsize
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

def get_speed(currentSpeed, acc_space):
    max = max_speed
    min = min_speed
    speeds = currentSpeed + acc_space * time_step
    for i in range(len(speeds)):
        if speeds[i] > max:
            speeds[i] = max
        elif speeds[i] < min:
            speeds[i] = min
    return speeds



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
    F[start] = heuristic(start, own_name) # ToDo 1: heuristic function

    closedVertices = set()
    openVertices = {start: [init_pos, init_head, 0, init_speed]}
    cameFrom = {}
    currentTime = 0
    currentSpeed = init_speed

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
                currentSpeed = openVertices[vertex][3]
        intruder_pos = []
        for name in intruder:
            if currentTime >= np.shape(intruder[name])[0]:
                intruder_pos.append(intruder[name][-1])
            else:
                intruder_pos.append(intruder[name][int(currentTime)])

        #if currentVertex[0] > x_range[1] or currentVertex[0] < x_range[0]:
        dis2end = world.point_dist(currentVertex, end_pos)
        if dis2end <= max_speed * time_step:
            #retrace our route backward
            path = [currentVertex]
            act = [currentHead * 180 / math.pi]
            speed = [currentSpeed]
            while currentVertex in cameFrom:
                currentAct = cameFrom[currentVertex][1]
                currentSpeed = cameFrom[currentVertex][2]
                currentVertex = cameFrom[currentVertex][0]
                path.append(currentVertex)
                act.append(currentAct * 180 / math.pi)
                speed.append(currentSpeed)
            act = list((np.array(act) + 0.01) // 1)
            path.reverse()
            act.reverse()
            speed.reverse()
            return path, act, speed #Done!

        #Mark the current vertex as closed.
        del(openVertices[currentVertex])
        closedVertices.add(currentVertex)

        #Update scores for vertices near the current position
        g_speed = get_speed(currentSpeed, acc_space) #************************TODO*************************
        neighbour = world.get_vertex_neighbours(currentPos, currentHead, action_space, currentTime, g_speed)#dict(neighbourVertex: [neighbourPos,neighbourHead])
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
            delta_angle = neighbour[neighbourVertex][1] - currentHead
            cameFrom[neighbourVertex] = [currentVertex, delta_angle, currentSpeed]
            G[neighbourVertex] = candidateG
            H = heuristic(currentPos, own_name)
            #H2 = chebishev_heuristic(neighbourPos, init_plan[own_name][1])
            #headCost = head_cst(neighbour[neighbourVertex][1], init_head)
            #actionCost = abs(delta_angle) * DD
            acc = (neighbour[neighbourVertex][3] - currentSpeed)/time_step
            accelarationCost = abs(acc) * BB
            F[neighbourVertex] = G[neighbourVertex] + H +accelarationCost
    raise RuntimeError("A* failed to find a solution")


def main():
    iter_step = 0
    path = {}
    for ac in init_plan:
        x = init_plan[ac][2][0] + math.cos(init_plan[ac][3]) * init_speed * np.arange(0, maxTimeStep + 100)#************************TODO*************************
        y = init_plan[ac][2][1] + math.sin(init_plan[ac][3]) * init_speed * np.arange(0, maxTimeStep + 100)#************************TODO*************************
        path[ac] = np.array(list(zip(x, y)))
    while iter_step < num_iteration:
        for ac in order:
            intruder_path = path.copy()
            del intruder_path[ac]
            path[ac], act, speed = AStarSearch(intruder_path, ac)
            plt.plot(speed)

            '''plt.plot(*zip(*path['ac1']), '-k.', *zip(*path['ac2']), '-r.', *zip(*path['ac3']), '-b.',*zip(*path['ac4']), '-g.')
            plt.ylim(y_range[0], y_range[1])
            plt.xlim(x_range[0], x_range[1])'''
            plt.savefig('{}iteration_aircraft{}'.format(iter_step, ac))
            plt.close()
        iter_step += 1

    plt.plot(*zip(*path['ac1']), '-k.', *zip(*path['ac2']), '-r.', *zip(*path['ac3']), '-b.', *zip(*path['ac4']), '-g.',[x_range[0], x_range[1]],[y_range[0], y_range[1]],'--y', [x_range[0], x_range[1]],[y_range[1], y_range[0]],'--y')
    plt.ylim(y_range[0], y_range[1])
    plt.xlim(x_range[0], x_range[1])
    plt.show()

x_range = np.array([-500, 500, 1])  # 1 indicates 0.01 nautical mile
y_range = np.array([-500, 500, 1])  # 1 indicates 0.01 nautical mile
time_step = 1  # seconds
init_speed = 20  # 1 indicates 0.01 nautical mile per second
max_speed = init_speed + 5
min_speed = init_speed - 5
action_space = np.array([0 * math.pi / 180, 0, 0 * math.pi / 180])  # heading angle in rad. Nagative sign indicates right hand of the aircraft.
action_cost = [0.1, 0]
acc_space = np.array([-1, 0, 1])

init_plan = {'ac1': [[1, 0], [x_range[1] - 100, y_range[1] - 100], [x_range[0], y_range[0]], 45 * math.pi / 180], 'ac2': [[-1, 0], [x_range[1] - 100, y_range[0] + 100], [x_range[0], y_range[1]], -45 * math.pi / 180], 'ac3': [[1, 0], [x_range[0] + 100, y_range[0] + 100], [x_range[1], y_range[1]], 225 * math.pi / 180], 'ac4': [[-1, 0], [x_range[0] + 100, y_range[1] - 100], [x_range[1], y_range[0]], 135 * math.pi / 180]}  # key: name of aircraft, value: [[a,b], waypoint in y=ax+b, [init_pos], init_head]
AA = -0.01
BB = 0.01
world = Grid(x_range, y_range)
order = ['ac1','ac2','ac3','ac4']
num_iteration = 5
maxTimeStep = 70
SEP = 25
gamma2 =2500

if __name__ == "__main__":
    main()