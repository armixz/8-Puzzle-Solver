# 
# +-----------------------------+
# | Homework #1                 |
# | Professor: Dr. Dan Moldovan |
# | CS 4365.002                 |
# |-----------------------------+
# | NAME : ARMIN ZIAEI          |
# | DATE : 09/28/2021           |
# | NETID: AXZ172330            |
# +-----------------------------+


# Objective:
#     Implement an 8-puzzle solver.

# Description:
#     The 8-puzzle problem is played on a 3-by-3 grid with 8 square tiles labeled 1 through 8 and
#     a blank tile. Your goal is to rearrange the blocks so that they are in the order shown below.
#     You are permitted to slide blocks horizontally or vertically into the blank tile.

#   It should have the following algorithms:
#     1. Depth-first search
#     2. Iterative deepening search
#     3. A* search using two different suitable heuristics


#      homework1.py
#------------------------------------------------------------------------------------------------

#------------------------------------------------------------------------------------------------
# --- Libraries --- #

import numpy as np
import os
import sys

#------------------------------------------------------------------------------------------------
# --- Checking Operating System --- #

def get_platform():
    platforms = {
        'linux1' : 'Linux',
        'linux2' : 'Linux',
        'darwin' : 'OS X',
        'win32' : 'Windows'
    }
    if sys.platform not in platforms:
        return sys.platform
    
    return platforms[sys.platform]

#------------------------------------------------------------------------------------------------
# --- Clear the screen --- #

def clear_screen():
    if (get_platform() == 'Windows'):
        os.system('cls')
    elif(get_platform() == 'linux' or get_platform() == 'OS X'):
        os.system('clear')

#------------------------------------------------------------------------------------------------
# --- Class Node --- #

class Node():
    def __init__(self, state, parent, action, depth, step_cost, path_cost, heuristic_cost):
        self.state          = state 
        self.parent         = parent                    # parent node
        self.action         = action                    # up, left, down, right
        self.depth          = depth                     # depth of the node in the tree
        self.step_cost      = step_cost                 # g(n)
        self.heuristic_cost = heuristic_cost            # h(n)
        self.path_cost      = path_cost                 # f(n) = g(n) + h(n)
        
        # --- children node
        self.move_up    = None 
        self.move_left  = None
        self.move_down  = None
        self.move_right = None

    #--------------------------------------------------------------------------------------------
    # --- Print Goal Trace --- #
    
    # --- trace back to the root node once the goal is found
    def print_path(self):
        
        # --- FILO stacks to place the trace
        state_trace             = [self.state]
        action_trace            = [self.action]
        depth_trace             = [self.depth]
        step_cost_trace         = [self.step_cost]
        path_cost_trace         = [self.path_cost]
        heuristic_cost_trace    = [self.heuristic_cost]
        
        # --- add a node as tracing tree
        while self.parent:
            self = self.parent
            state_trace.append(self.state)
            action_trace.append(self.action)
            depth_trace.append(self.depth)
            step_cost_trace.append(self.step_cost)
            heuristic_cost_trace.append(self.heuristic_cost)
            path_cost_trace.append(self.path_cost)
            
        # --- print out the path
        step_counter = 0
        while state_trace:
            print('\n', state_trace.pop())
            print('  depth=', str(depth_trace.pop()), '\taction=', action_trace.pop())

            step_counter += 1

    #--------------------------------------------------------------------------------------------
    # --- is_valid_moves functions --- #
    
    # --- if moving down is valid
    def is_valid_down(self):
        # --- index of the empty tile
        zero_index=[i[0] for i in np.where(self.state==0)] 
        if zero_index[0] == 0: return False
        else:
            up_value = self.state[zero_index[0]-1, zero_index[1]] # top tile value
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = up_value
            new_state[zero_index[0]-1, zero_index[1]] = 0
            return new_state, up_value
        
    # --- if moving right is valid
    def is_valid_right(self):
        zero_index=[i[0] for i in np.where(self.state==0)] 
        if zero_index[1] == 0: return False
        else:
            left_value = self.state[zero_index[0], zero_index[1]-1] # left tile value
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = left_value
            new_state[zero_index[0], zero_index[1]-1] = 0
            return new_state, left_value
        
    # --- if moving up is valid
    def is_valid_up(self):
        zero_index=[i[0] for i in np.where(self.state==0)] 
        if zero_index[0] == 2: return False
        else:
            lower_value = self.state[zero_index[0]+1, zero_index[1]] # bottom tile value
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = lower_value
            new_state[zero_index[0]+1, zero_index[1]] = 0
            return new_state, lower_value
        
    # --- if moving left is valid
    def is_valid_left(self):
        zero_index=[i[0] for i in np.where(self.state==0)] 
        if zero_index[1] == 2: return False
        else:
            right_value = self.state[zero_index[0], zero_index[1]+1] # right tile value
            new_state = self.state.copy()
            new_state[zero_index[0], zero_index[1]] = right_value
            new_state[zero_index[0], zero_index[1]+1] = 0
            return new_state, right_value
    
    #--------------------------------------------------------------------------------------------
    # --- DFS --- #
    
    def dfs(self, goal_state):

        queue = [self]              # FILO
        queue_num_nodes_popped = 0  # nodes: popped the queue
        queue_max_length = 1        # nodes: max in the queue
        
        depth_queue = [0]           # queue: node depth
        path_cost_queue = [0]       # queue: path cost
        visited = set([])           # visited states
        
        while queue:
            # --- maximum length of the queue
            if len(queue) > queue_max_length:
                queue_max_length = len(queue)
                
            current_node = queue.pop(0)                             
            queue_num_nodes_popped += 1 
            
            current_depth = depth_queue.pop(0)                      
            current_path_cost = path_cost_queue.pop(0)              
            visited.add(tuple(current_node.state.reshape(1,9)[0])) 

            if(current_depth < 10):
                # --- when the goal state is found
                if np.array_equal(current_node.state,goal_state):
                    current_node.print_path()
                    print("\nNumber of moves = ", current_depth)
                    print("Number of states enqueued = ", queue_max_length)
                    return True
                
                else:                
                    # --- if moving down is valid
                    if current_node.is_valid_down():
                        new_state,up_value = current_node.is_valid_down()
                        # --- if the node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            # --- create a new child node
                            current_node.move_down = Node(state=new_state,
                                                        parent=current_node,
                                                        action='down',
                                                        depth=current_depth+1,
                                                        step_cost=up_value,
                                                        path_cost=current_path_cost+up_value,
                                                        heuristic_cost=0)

                            queue.insert(0,current_node.move_down)
                            depth_queue.insert(0,current_depth+1)
                            path_cost_queue.insert(0,current_path_cost+up_value)
                        
                    # --- if moving left is valid
                    if current_node.is_valid_right():
                        new_state,left_value = current_node.is_valid_right()
                        # --- if the node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            # --- create a new child node
                            current_node.move_right = Node(state=new_state,
                                                            parent=current_node,
                                                            action='right',
                                                            depth=current_depth+1,
                                                            step_cost=left_value,
                                                            path_cost=current_path_cost+left_value,
                                                            heuristic_cost=0)

                            queue.insert(0,current_node.move_right)
                            depth_queue.insert(0,current_depth+1)
                            path_cost_queue.insert(0,current_path_cost+left_value)
                    
                    # --- if moving up is valid
                    if current_node.is_valid_up():
                        new_state,lower_value = current_node.is_valid_up()
                        # --- if the node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            # --- create a new child node
                            current_node.move_up = Node(state=new_state,
                                                        parent=current_node,
                                                        action='up',
                                                        depth=current_depth+1,
                                                        step_cost=lower_value,
                                                        path_cost=current_path_cost+lower_value,
                                                        heuristic_cost=0)

                            queue.insert(0,current_node.move_up)
                            depth_queue.insert(0,current_depth+1)
                            path_cost_queue.insert(0,current_path_cost+lower_value)

                    # --- if moving right is valid
                    if current_node.is_valid_left():
                        new_state,right_value = current_node.is_valid_left()
                        # --- if the node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            # --- create a new child node
                            current_node.move_left = Node(state=new_state,
                                                        parent=current_node,
                                                        action='left',
                                                        depth=current_depth+1,
                                                        step_cost=right_value,
                                                        path_cost=current_path_cost+right_value,
                                                        heuristic_cost=0)

                            queue.insert(0,current_node.move_left)
                            depth_queue.insert(0,current_depth+1)
                            path_cost_queue.insert(0,current_path_cost+right_value)
            else:
                print('goal state was not found before or at depth 10.')
                break
    #--------------------------------------------------------------------------------------------
    # --- IDS --- #

    def ids(self, goal_state):
        queue_num_nodes_popped = 0  # nodes: popped the queue
        queue_max_length = 1        # nodes: max in the queue
        
        queue = [self]              # FILO
        depth_queue = [0]           # queue: depth
        path_cost_queue = [0]       # queue: path cost
        visited = set([])           # visited states

        while queue:
            # --- max length of the queue
            if len(queue) > queue_max_length:
                queue_max_length = len(queue)

            current_node = queue.pop(0) 
            queue_num_nodes_popped += 1 

            current_depth = depth_queue.pop(0)
            current_path_cost = path_cost_queue.pop(0)
            visited.add(tuple(current_node.state.reshape(1,9)[0])) 

            # --- trace back to the root node once the goal is found
            if np.array_equal(current_node.state,goal_state):
                current_node.print_path()
                print("\nNumber of moves = ", current_depth)
                print("Number of states enqueued = ", queue_max_length)
                return True

            else:              
                if current_depth < 10:
                    # --- if moving down is valid
                    if current_node.is_valid_down():
                        new_state,up_value = current_node.is_valid_down()
                        # --- if node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            # --- create a child node
                            current_node.move_down = Node(state = new_state,
                                                            parent = current_node,
                                                            action = 'down',
                                                            depth  = current_depth+1,
                                                            step_cost = up_value,
                                                            path_cost = current_path_cost+up_value,
                                                            heuristic_cost = 0)

                            queue.insert(0,current_node.move_down)
                            depth_queue.insert(0,current_depth+1)
                            path_cost_queue.insert(0,current_path_cost+up_value)

                    # --- if moving right is valid
                    if current_node.is_valid_right():
                        new_state,left_value = current_node.is_valid_right()
                        # --- if node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            # --- create a child node
                            current_node.move_right = Node(state = new_state,
                                                            parent = current_node,
                                                            action = 'right',
                                                            depth  = current_depth+1,
                                                            step_cost = left_value,
                                                            path_cost = current_path_cost+left_value,
                                                            heuristic_cost = 0)

                            queue.insert(0,current_node.move_right)
                            depth_queue.insert(0,current_depth+1)
                            path_cost_queue.insert(0,current_path_cost+left_value)

                    # --- if moving up is valid
                    if current_node.is_valid_up():
                        new_state,lower_value = current_node.is_valid_up()
                        # --- if the node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            # --- create a child node
                            current_node.move_up = Node(state  = new_state,
                                                        parent = current_node,
                                                        action = 'up',
                                                        depth  = current_depth+1,
                                                        step_cost = lower_value,
                                                        path_cost = current_path_cost+lower_value,
                                                        heuristic_cost = 0)

                            queue.insert(0,current_node.move_up)
                            depth_queue.insert(0,current_depth+1)
                            path_cost_queue.insert(0,current_path_cost+lower_value)

                    # --- if moving left is valid
                    if current_node.is_valid_left():
                        new_state,right_value = current_node.is_valid_left()
                        # --- if the node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            # --- create a child node
                            current_node.move_left = Node(state = new_state,
                                                            parent = current_node,
                                                            action = 'left',
                                                            depth  = current_depth+1,
                                                            step_cost = right_value,
                                                            path_cost = current_path_cost+right_value,
                                                            heuristic_cost = 0)

                            queue.insert(0,current_node.move_left)
                            depth_queue.insert(0,current_depth+1)
                            path_cost_queue.insert(0,current_path_cost+right_value)
                
                else:
                    print('goal state was not found before or at depth 10.')
                    break
    #--------------------------------------------------------------------------------------------
    # --- astar1, astar2 --- #
    
    # --- return heuristic cost (astar1, astar2)
    def get_h_cost(self, new_state, goal_state, heuristic_function, path_cost, depth):
        if heuristic_function == 'num_misplaced':
            return self.h_misplaced_cost(new_state,goal_state)
        elif heuristic_function == 'manhattan':
            return self.h_manhattan_cost(new_state,goal_state)

    # --- return heuristic cost (astar1)
    def h_misplaced_cost(self,new_state,goal_state):
        cost = np.sum(new_state != goal_state)-1 # minus 1 to exclude the empty tile
        if cost > 0: return cost
        else: return 0 # when all tiles matches
    
    # --- return heuristic cost (astar2)
    def h_manhattan_cost(self,new_state,goal_state):
        current = new_state
        # --- positioning the digits
        goal_position_dic = {1:(0,0), 2:(0,1), 3:(0,2), 8:(1,0), 0:(1,1), 4:(1,2), 7:(2,0), 6:(2,1), 5:(2,2)} 
        sum_manhattan = 0
        for i in range(3):
            for j in range(3):
                if current[i,j] != 0:
                    sum_manhattan += sum(abs(a-b) for a,b in zip((i,j), goal_position_dic[current[i,j]]))
        return sum_manhattan


    #--------------------------------------------------------------------------------------------     
    # --- Heuristic --- #
    
    def astar(self,goal_state,heuristic_function):
        queue = [(self,0)]          
        queue_num_nodes_popped = 0  # nodes: popped the queue
        queue_max_length = 1        # nodes: max in the queue
        
        depth_queue = [(0,0)]       # queue: depth
        path_cost_queue = [(0,0)]   # queue: path cost
        visited = set([])           # visited states

        while queue:
            # sort queue: path_cost+heuristic cost
            queue = sorted(queue, key=lambda x: x[1])
            depth_queue = sorted(depth_queue, key=lambda x: x[1])
            path_cost_queue = sorted(path_cost_queue, key=lambda x: x[1])
            
            # max length of the queue
            if len(queue) > queue_max_length:
                queue_max_length = len(queue)
                
            current_node = queue.pop(0)[0] 
            queue_num_nodes_popped += 1 
            current_depth = depth_queue.pop(0)[0] 
            current_path_cost = path_cost_queue.pop(0)[0] 
            visited.add(tuple(current_node.state.reshape(1,9)[0]))
            
            # --- trace back to the root node once the goal is found
            if np.array_equal(current_node.state,goal_state):
                current_node.print_path()
                print("\nNumber of moves = ", current_depth)
                print("Number of states enqueued = ", queue_max_length)
                return True
            
            else:     
                if current_depth < 10:
                    # --- if moving down is valid
                    if current_node.is_valid_down():
                        new_state,up_value = current_node.is_valid_down()
                        # --- if the node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            path_cost=current_path_cost+up_value
                            depth = current_depth+1
                            # --- get heuristic cost
                            h_cost = self.get_h_cost(new_state,goal_state,heuristic_function,path_cost,depth)
                            # --- create a child node
                            total_cost = path_cost+h_cost
                            current_node.move_down = Node(state=new_state,
                                                            parent=current_node,
                                                            action='down',
                                                            depth=depth,
                                                            step_cost=up_value,
                                                            path_cost=path_cost,
                                                            heuristic_cost=h_cost)

                            queue.append((current_node.move_down, total_cost))
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append((path_cost, total_cost))
                        
                    # see if moving left tile to the right is a valid move
                    if current_node.is_valid_right():
                        new_state,left_value = current_node.is_valid_right()
                        # check if the resulting node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            path_cost=current_path_cost+left_value
                            depth = current_depth+1
                            # get heuristic cost
                            h_cost = self.get_h_cost(new_state,goal_state,heuristic_function,path_cost,depth)
                            # create a new child node
                            total_cost = path_cost+h_cost
                            current_node.move_right = Node(state=new_state,
                                                            parent=current_node,
                                                            action='right',
                                                            depth=depth,
                                                            step_cost=left_value,
                                                            path_cost=path_cost,
                                                            heuristic_cost=h_cost)

                            queue.append((current_node.move_right, total_cost))
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append((path_cost, total_cost))
                        
                    # --- if moving up is valid
                    if current_node.is_valid_up():
                        new_state,lower_value = current_node.is_valid_up()
                        # --- if the node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            path_cost=current_path_cost+lower_value
                            depth = current_depth+1
                            # --- get heuristic cost
                            h_cost = self.get_h_cost(new_state,goal_state,heuristic_function,path_cost,depth)
                            # --- create a child node
                            total_cost = path_cost+h_cost
                            current_node.move_up = Node(state=new_state,
                                                        parent=current_node,
                                                        action='up',
                                                        depth=depth,
                                                        step_cost=lower_value,
                                                        path_cost=path_cost,
                                                        heuristic_cost=h_cost)

                            queue.append((current_node.move_up, total_cost))
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append((path_cost, total_cost))

                    # --- if moving left is valid
                    if current_node.is_valid_left():
                        new_state,right_value = current_node.is_valid_left()
                        # --- if the node is already visited
                        if tuple(new_state.reshape(1,9)[0]) not in visited:
                            path_cost=current_path_cost+right_value
                            depth = current_depth+1
                            # --- get heuristic cost
                            h_cost = self.get_h_cost(new_state,goal_state,heuristic_function,path_cost,depth)
                            # --- create a child node
                            total_cost = path_cost+h_cost
                            current_node.move_left = Node(state=new_state,
                                                            parent=current_node,
                                                            action='left',
                                                            depth=depth,
                                                            step_cost=right_value,
                                                            path_cost=path_cost,
                                                            heuristic_cost=h_cost)

                            queue.append((current_node.move_left, total_cost))
                            depth_queue.append((depth, total_cost))
                            path_cost_queue.append((path_cost, total_cost))
                else:
                    print('goal state was not found before or at depth 10.')
                    break

#--------------------------------------------------------------------------------------------
# --- Driver --- #

def main(argv):

    # --- Clear Console --- #
    clear_screen()

    if len(sys.argv) == 3:        
        algorithm_name  = sys.argv[1]
        input_file_path = sys.argv[2]   
    else:
        print('Pass 2 Arguments: python homework1.py <algorithm_name> <input_file_path>\n')
        quit()


    file = open(input_file_path, 'r')
    goal_state = np.array([1,2,3,4,0,5,6,7,8]).reshape(3,3)
    
    i = 1
    while True:

        # --- read line-by-line --- #
        line = file.readline()
            
        # --- break if end of input_file --- #
        if not line: 
            break

        print('line',i,' -> test_case = ', line) 
        
        # --- read line from input_file and convert it to numpy --- #
        cmd_init_state = [int(i) for i in line.split(' ')]
        init_state = np.array(cmd_init_state).reshape(3,3)    

        # --- Create root Node --- #
        root_node = Node(state=init_state,
                        parent=None,
                        action=None,
                        depth=0,
                        step_cost=0,
                        path_cost=0,
                        heuristic_cost=0)

        # --- DFS (stack) --- #
        if(algorithm_name == 'dfs'):
            root_node.dfs(goal_state)   
        
        # --- IDS: BFS + DFS (stack) --- #
        elif(algorithm_name == 'ids'):
            root_node.ids(goal_state)   
        
        # --- A*1 : path_cost + heuristic_cost (priority queue) --- #
        elif(algorithm_name == 'astar1'):
            root_node.astar(goal_state,heuristic_function = 'num_misplaced')    
        
        # --- A*2 : path_cost + heuristic_cost (priority queue) --- #
        elif(algorithm_name == 'astar2'):
            root_node.astar(goal_state,heuristic_function = 'manhattan') 

        else:
            print('available <algorithm_name> = [dfs, ids, astar1, astar2]\n')
        
        print('-'*60)
        i = i + 1
    file.close()
    
#--------------------------------------------------------------------------------------------
# --- main module --- #

if __name__ == '__main__':
    main(sys.argv)