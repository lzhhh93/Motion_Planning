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


def track_the_best_plan(idx = 2):
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
    
def RTDP():
    itr_num = 0
    bellman_error = np.inf
    bellman_error_list = []
    #Gmap = deepcopy(race_track).astype(float) 

    #print(key,state.g_value)
    while bellman_error > 0.0001:
        itr_num += 1
        
        #Choice 1
        rand_start = START_LINE[np.random.randint(low=0, high=4, size=1)[0]]
        #with 0 as first element of rand_start and pick the second one randomly with [3,6]
        
        #Choice 2
        #rand_start = [0,5]
        
        
        state = Node(rand_start[0],rand_start[1],0,0)
        
        state = graph[state.key]
        route = []
        route.append( state.key )
        
        t_route = []
        
        while not state.is_goal:
            
            next_prob_key = []
            next_prob_g = []
            
            for key in state.next_prob_9:
            
                nx_state = graph[key]
                px = nx_state.px
                py = nx_state.py
                vx = nx_state.vx
                vy = nx_state.vy
                px_b = px + vx
                
                if vx == 1:
                    npx = px + vx - 1
                elif vx == 0:
                    npx = px
                else:
                    npx = px + vx -1 + vx - 2
                    
                if vy == 1:
                    npy = py + vy -1
                elif vy == 0:
                    npy = py
                else:
                    npy = py + vy -1 + vy - 2
                 
                if 0 <= px <= 3 and 3 <= py <= 6:
                    
                    y_bound_0 = 3
                    y_bound_1 = 6
                    x_bound_0 = 1
                    
                    if y_bound_0 > npy or npy > y_bound_1:
                        #nx_state.g_value += 99
                        continue
                      
                if 4 <= px <= 7 and 2 <= py <= 6:
                    
                    y_bound_0 = 2
                    y_bound_1 = 6
                    x_bound_0 = 5
                    
                    if y_bound_0 > npy or npy > y_bound_1:
                        #nx_state.g_value += 99
                        continue
                    elif x_bound_0 > px_b:
                           continue
                      
                if 8 <= px <= 11 and 1 <= py <= 6:
                    
                    y_bound_0 = 1
                    y_bound_1 = 6
                    x_bound_0 = 9
                    
                    if y_bound_0 > npy and npy > y_bound_1:
                        #nx_state.g_value += 99
                        continue
                    elif x_bound_0 > px_b:
                           continue
                
                if 12 <= px <= 31 and 0 <= py <= 6:
                    
                    y_bound_0 = 0
                    y_bound_1 = 6
                    x_bound_0 = 13
                    x_bound_1 = 34
                    n_npx = npx + vx - 3
                    
                    if y_bound_0 > npy or npy > y_bound_1 or x_bound_0 > npx or npx > x_bound_1:
                        #nx_state.g_value += 99
                        continue
                    elif x_bound_0 > px_b:
                        continue
                    
                    if n_npx > x_bound_1:
                        #nx_state.g_value += 99
                        continue
                
                if 32 <= px <= 34 and 0 <= py <= 11:

                    y_bound_0 = 0
                    y_bound_1 = 11
                    x_bound_0 = 32
                    x_bound_1 = 34
                    n_npx = npx + vx - 3
                    
                    if y_bound_0 > npy or npy > y_bound_1 or x_bound_0 > npx or npx > x_bound_1:
                        #nx_state.g_value += 99
                        continue
                    
                    if n_npx > x_bound_1:
                        #nx_state.g_value += 99
                        continue
                
                if  nx_state.px == 0 and 3 <= nx_state.py <= 6:
                    continue
            
                if nx_state.key in route:
                    continue 
                
                
                
                next_prob_key.append(key)
                next_prob_g.append(nx_state.g_value)
            
            if next_prob_g == []:
                rand_start = START_LINE[np.random.randint(low=0, high=3, size=1)[0]]
        
                state = Node(rand_start[0],rand_start[1],0,0)
        
                state = graph[state.key]
                route = []
                route.append( state.key )
                continue
            
            min_idx = np.argmin(next_prob_g)
            route.append(next_prob_key[min_idx])
            t_route.append(next_prob_key[min_idx])
            state = graph[ route[-1] ]
        
        expected_val_ary = []  
        bellman_error = 0.0
        for key in route:
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
        bellman_error_list.append(bellman_error)
        print("{}th iteration: {}".format(itr_num, bellman_error))
    plt.figure()
    x_axis = range(len(bellman_error_list))
    plt.plot(x_axis, bellman_error_list)
    plt.show()

def G_Initialization(graph):
    for key in graph.keys():
        
        state = graph[key]
        px = state.px
        py = state.py
        #vx = state.vx
        #vy = state.vy
        
        goal_x = 32
        goal_y = 11
        dx = abs(px - goal_x)
        dy = abs(py - goal_y)
        #if  px == 33 and py == 6 and state.vx ==3 and state.vy ==0:
        #    counter =1
         
        #if px == 0 and py == 3 and state.vx ==0 and state.vy ==0:
        #    counter =1
        
        if track_map[px][py] == 0: 
            
            if px < 32:
            #Zone 1
                
                #Diagonal Dist:
                #state.g_value = (dx + dy - 2*min(dx,dy))/4 + min(dx,dy)/4
                
                #Euclidean Dist:
                state.g_value = np.sqrt( pow(px-goal_x,2) + pow(py-goal_y,2) )
                
                #Manhatten Dist:
                #state._g_value = abs(px - goal_x) + abs(py -goal_y)
                
                #Gmap[px][py] = state.g_value

            #Zone 2    
            elif 31 < px :
                  
                  
                  #Diagonal Dist:
                  #state.g_value = abs(py - goal_y)
                  #state.g_value = (dx + dy - 2*min(dx,dy))/4 + min(dx,dy)/4
                  
                  #Euclidean Dist:
                  state.g_value = abs(py - goal_y)
                  
                  #Manhatten Dist:
                  #state._g_value = abs(py -goal_y)
                  
                  
                  #Gmap[px][py] = state.g_value                    
            
        if track_map[px][py] == 2:
            
            #Diagonal Dist:
            #state.g_value = (dx + dy - 2*min(dx,dy))/4 + min(dx,dy)/4
            
            #Euclidean Dist:
            state.g_value = np.sqrt( pow(px-goal_x,2) + pow(py-goal_y,2) )
            
            #Manhatten Dist:
            #state._g_value = abs(px - goal_x) + abs(py -goal_y)
            
            #Gmap[px][py] = state.g_value 
            
        if track_map[px][py] == 3:
            state.g_value = 0

if __name__ == '__main__':
    path = './solution_w/graph_rtdp.dat'
    track_map = race_track
    #build_up_graph(track_map, path)
    graph = pickle.load(open(path, 'rb')) 
    G_Initialization(graph)
    
    #for key in graph.keys():
    #    state = graph[key]
    #    print(key)
    #    print("with")
    #    print(graph[key].g_value)
    # solve
    #dynamic_programming()
    RTDP()
    plan = track_the_best_plan()
    visualize_the_best_plan(plan, track_map)
