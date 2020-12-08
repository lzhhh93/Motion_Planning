import pickle
from racetracks import *
from graph_node import Node
import matplotlib.pyplot as plt
from copy import deepcopy 
import math

seed = np.random.seed(1234)
graph = {}



def build_up_graph(grid, save_path):
    max_vel = 5

    # velocity dimension
    vel_list = []
    for i_vel in range(-max_vel+1, max_vel):
        for j_vel in range(-max_vel+1, max_vel):
            vel_list.append([i_vel, j_vel])

    # position dimension
    x_idx, y_idx = np.where(grid == FREE)
    # FREE is the variable in racetracks.py 
    coord = np.stack([x_idx, y_idx], axis=1)
    for p_idx in range(coord.shape[0]):
        pnt = coord[p_idx]
        for vel in vel_list:
            #if pnt[0] == 30 and pnt[1] == 6 and vel[0] == 4 and vel[1] ==0:
            #   counter =1
            state = Node(pnt[0], pnt[1], vel[0], vel[1])
            state.connect_to_graph(grid)
            # add all the key of next waypoints caused by v to next_prob_9 and next_prob_1
           
            graph[state.key] = state

    for pnt in START_LINE:
        state = Node(pnt[0], pnt[1], 0, 0)
        state.connect_to_graph(grid)
        graph[state.key] = state

    for pnt in FINISH_LINE:
        state = Node(pnt[0], pnt[1], 0, 0)
        state.is_goal = True
        graph[state.key] = state

    output = open(save_path, 'wb')
    pickle.dump(graph, output)



def check_graph(grid):
    plt.figure(figsize=(4.5, 16))
    plt.pcolor(grid, edgecolors='k', linewidths=1)
    for key in graph.keys():
        for child_idx, child_key in enumerate(graph[key].next_prob_1): # or next_prob_1
            ux, uy = ACTION_SPACE[child_idx]
            vx, vy = graph[key].vx + ux,  graph[key].vy + uy
            child = graph[child_key]
            # check a specific connection
            # plt.title(str(vy) + '_' + str(vx))
            # plt.show()
            if [child.px, child.py] in START_LINE:
                print('found')
                continue
            plt.arrow(graph[key].py + 0.5, graph[key].px + 0.5,
                      child.py - graph[key].py, child.px - graph[key].px,
                      color='r', head_width=0.3, head_length=0.1)
            print(key, child_idx)
        # end for
    # end for
    plt.show()


def track_the_best_plan(idx = 0):
    start_node = Node(START_LINE[idx][0], START_LINE[idx][1], 0, 0)
    start_key = start_node.key
    state = graph[start_key]
    trajectory = [state]
    print(state.px, state.py, state.vx, state.vy)
    # for i in range(grid.shape[0]+grid.shape[1]) a safer condition
    while not state.is_goal:
        value_uk = []
        for child_idx in range(len(ACTION_SPACE)):     
            # 0-17
            child_key_9 = state.next_prob_9[child_idx]
            child_9 = graph[child_key_9]
            value_uk.append(child_9.g_value)
        child_key = state.next_prob_9[np.argmin(value_uk)]
        # find the key that argmin(value_uk)
        state = graph[child_key]
        trajectory.append(state)
        print(state.px, state.py, state.vx, state.vy)
    return trajectory



def visualize_the_best_plan(plan, grid_para):
    assert isinstance(plan, list)
    plt.figure(figsize=(4.5, 16))
    plt.pcolor(grid_para, edgecolors='k', linewidths=1)
    plan_len = len(plan)
    plan.append(plan[-1])
    for i in range(plan_len):
        plt.arrow(plan[i].py + 0.5, plan[i].px + 0.5,
                  plan[i+1].py - plan[i].py, plan[i+1].px - plan[i].px,
                  color='r', head_width=0.3, head_length=0.1)
    plt.show()


def dynamic_programming():
    itr_num = 0
    bellman_error = np.inf
    bellman_error_list = []
    while bellman_error > 0.0001:
        itr_num += 1
        bellman_error = 0.0
        for key in graph.keys():
            state = graph[key]
            if state.is_goal:
                state.g_value = 0
            else:
                value_uk = []
                for child_idx in range(len(ACTION_SPACE)):
                    child_key_9 = state.next_prob_9[child_idx]
                    child_9 = graph[child_key_9]
                    child_key_1 = state.next_prob_1[child_idx]
                    child_1 = graph[child_key_1]
                    expected_cost_uk = 0.9 * (1 + child_9.g_value) + 0.1 * (1 + child_1.g_value)
                    value_uk.append(expected_cost_uk)
                current_value = min(value_uk)
                bellman_error += np.linalg.norm(state.g_value - current_value)
                state.g_value = min(value_uk)
            # end if
        # end for
        bellman_error_list.append(bellman_error)
        print("{}th iteration: {}".format(itr_num, bellman_error))
    # end while

    plt.figure()
    x_axis = range(len(bellman_error_list))
    plt.plot(x_axis, bellman_error_list)
    plt.show()

def explore_action(u_idx,epsilon=0.8):
    if np.random.uniform(0,1)<epsilon:
        return np.random.randint(0,len(ACTION_SPACE))
    else:
        return u_idx
    
def RTDP():
    itr_num = 0
    bellman_error = np.inf
    bellman_error_list = []
    #Gmap = deepcopy(race_track).astype(float) 

    #print(key,state.g_value)
    while bellman_error > 0.0001:
        itr_num += 1
        
        rand_start = START_LINE[np.random.randint(low=0, high=3, size=1)[0]]
        
        state = Node(rand_start[0],rand_start[1],0,0)
        
        state = graph[state.key]
        route = []
        route.append( state.key )
        
        while not state.is_goal:

            child_g_list = []            

            for key in state.next_prob_9:
                child_9 = graph[key]
                child_g_list.append( child_9.g_value )
                        
            min_idx = np.argmin( child_g_list )
            min_idx = explore_action(min_idx)
            #choose child randomly or optimally
            
            route.append(state.next_prob_9[min_idx])
            state = graph[ route[-1] ]
        
        bellman_error = 0.0
        for key in route:
            
            expected_val_ary = []  
            state = graph[key]
            
            if not state.is_goal:
               current_value = state.g_value
               for idx in range(len(ACTION_SPACE)):
                   child_9 = graph[state.next_prob_9[idx]]
                   child_1 = graph[state.next_prob_1[idx]]
                   expected_value = (1+child_9.g_value)*0.9 + (1+child_1.g_value)*0.1
                   expected_val_ary.append(expected_value)
               state.g_value = min(expected_val_ary)
               bellman_error += abs(current_value - state.g_value)
            else:
                state.g_value=0

        bellman_error_list.append(bellman_error)
        print("{}th iteration: {}".format(itr_num, bellman_error))
    plt.figure()
    x_axis = range(len(bellman_error_list))
    plt.plot(x_axis, bellman_error_list)
    plt.show()

def G_Initialization(graph):
    goal_x = 32
    goal_y = 11
    for key in graph.keys():
        
        state = graph[key]
        px = state.px
        py = state.py
        vx = state.vx
        vy = state.vy
        v_m = max(vx,vy) #:not working. iteration sucks after 1st time
        v_s = abs(vx+vy)
        if v_s == 0 or v_m ==0:
            v_m = 1
            v_s = 1
                 
        if state.px not in [32,33,34] and state.py != 11:
           
           #M
           #state.g_value = abs(state.px-goal_x) + abs(state.py-goal_y) 
           ##modified: results of convergence and traj are the same, 228 th and 12 nodes
           #state.g_value = ( abs(state.px-goal_x) + abs(state.py-goal_y) )/v_s
           
           # E:
           #state.g_value = np.sqrt( pow(px-goal_x,2) + pow(py-goal_y,2))
           ## modified: results of convergence and traj are the same, 140 th and 13 nodes
           state.g_value =  ( np.sqrt( pow(px-goal_x,2) + pow(py-goal_y,2) ) )/v_s
           
           # D:
           #dx = abs(state.px-32)
           #dy = abs(state.py-11)
           #state.g_value = (dx + dy - 2*min(dx,dy))/4 + min(dx,dy)/4
           ## modified
           #state.g_value = ( (dx + dy - 2*min(dx,dy))/4 + min(dx,dy)/4 )/v_m
           
if __name__ == '__main__':
    path = './solution_wr/graph_rtdp.dat'
    track_map = race_track
    #build_up_graph(track_map, path)
    graph = pickle.load(open(path, 'rb')) 
    G_Initialization(graph)

    # solve
    #dynamic_programming()
    RTDP()
    plan = track_the_best_plan()
    visualize_the_best_plan(plan, track_map)
