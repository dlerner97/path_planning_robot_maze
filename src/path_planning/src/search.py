#<=============================== Imports ===================================>#
import math
import heapq
import numpy as np
from time import time
from search_base import Search

#<=============================== BFSSearch Class Definition ===================================>#
class BFSSearch(Search):
    
    """
    Breadth-First Search Class
    Parent: Search
    
    arg: map, action set dict, scale percent, add frame frequency, framerate
    
    Executes a breadth first searchon the given map with the given action set.
    """
    
    def __init__(self, map, action_set, scale_percent=100, add_frame_frequency=100, framerate=100, save_video=True):
        super().__init__(map, action_set, cost_algorithm=False, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate, save_video=save_video)

    # Grabs next node
    def _get_next_branch(self, parent, pos, action): 
         
        # Check if new position is on the grid
        on_grid =             pos[1] < self.height
        on_grid = on_grid and pos[1] >= 0
        on_grid = on_grid and pos[0] < self.width
        on_grid = on_grid and pos[0] >= 0 
        
        # Check if move is legal and generate that new state
        if (on_grid):
            loc_string = Search.pos2str(pos) 
            theta = 0
            if not self.discrete:
                theta = pos[2]
            
            parent_loc_string = Search.pos2str(parent)
            
            try:
                attempt = self.node_info[parent_loc_string][parent[2]]
            except KeyError:
                return False
            
            # If position corresponds to the winning state, return True
            node_info = self.node_info.get(loc_string, None)
            if self.at_goal_pos(pos):
                if node_info == None:
                    self.node_info[loc_string] = {}
                self.node_info[loc_string][theta] = {"parent": parent, "action": action}
                self.winning_loc = pos
                return True 
            
            # Check if the node is on an obstacle
            if self.no_obstacles(pos):
                
                # Check if the node has not been visited 
                if self.not_visited(node_info, theta):
                    
                    # Change pixel color to blue unless it overlaps with the start and end depictions
                    self.draw_on_grid(parent, pos, (255,0,0))
                    
                    # Update queue and parent info
                    if node_info == None:
                        self.node_info[loc_string] = {}
                    self.node_info[loc_string][theta] = {"parent": parent, "action": action}
                    self.q_set(pos)                
        return False
    
    # Check all subsequent directions for a given position
    def _check_subsequent_nodes(self, parent):

        # Check all directions
        found_goal = False
        for branch_info in self.action_set.values(): 
            # Get the new position using tuple addition
            pos = self.get_next_pos(parent, branch_info['move'])
            found_goal = self._get_next_branch(parent, pos, branch_info['move']) or found_goal
        
        # If any of these are true, we have reached the goal state. Propogate the true to the next level
        return found_goal
    
    # Find path
    def find_path(self):
        self._find_path()

#<=============================== DijkstraSearch Class Definition ===================================>#
class DijkstraSearch(Search):
    
    """
    Dijkstra Search Class
    Parent: Search
    
    arg: map, action set dict, scale percent, add frame frequency, framerate
    
    Executes a djikstra search on the given map with the given action set.
    """
    
    def __init__(self, map, action_set, scale_percent=100, add_frame_frequency=100, framerate=100, save_video=True):
        super().__init__(map, action_set, cost_algorithm=True, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate, save_video=save_video)

    # Grabs next node
    def _get_next_branch(self, parent, pos, action, branch_cost): 
         
        # Get the new position using tuple addition
        parent = parent[1]

        # Check if new position is on the grid
        on_grid =             pos[1] < self.height
        on_grid = on_grid and pos[1] >= 0
        on_grid = on_grid and pos[0] < self.width
        on_grid = on_grid and pos[0] >= 0 
        
        # Check if move is legal and generate that new state
        if (on_grid):
            loc_string = Search.pos2str(pos) 
            theta = 0
            if not self.discrete:
                theta = pos[2]
            
            parent_loc_string = Search.pos2str(parent)
            
            # Calculate cost
            try:
                parent_cost = self.node_info[parent_loc_string][parent[2]]["cost_from_first"]
            except KeyError:
                return False
                    
            cost_from_first = parent_cost+branch_cost
            node_info = self.node_info.get(loc_string, None)
            
            # If position corresponds to the winning state, return True
            if self.at_goal_pos(pos):
                if node_info == None:
                    self.node_info[loc_string] = {}
                self.node_info[loc_string][theta] = {"parent": parent, "action": action, "cost_from_first": cost_from_first}
                self.winning_loc = pos
                return True 
                       
            # Check if the node is on an obstacle
            if self.no_obstacles(pos):
                # Check if the node has already been visited
                
                if self.not_visited(node_info, theta):
                    # Change pixel color to blue unless it overlaps with the start and end depictions
                    self.draw_on_grid(parent, pos, (255,0,0))
                    
                    # Update queue and parent info
                    if node_info == None:
                        self.node_info[loc_string] = {}
                    self.node_info[loc_string][theta] = {"parent": parent, "action": action, "cost_from_first": cost_from_first}
                    self.q_set(cost_from_first, pos)
                    return False
                
                # Replace parent node if smaller cost path can be found
                keys = list(node_info.keys())
                closest_theta = keys[np.argmin([abs(theta-theta_i) < self.angle_threshold for theta_i in keys])]
                if cost_from_first < node_info[closest_theta]["cost_from_first"]:
                    self.node_info[loc_string].pop(closest_theta)
                    self.node_info[loc_string][theta] = {"parent": parent, "action": action, "cost_from_first": cost_from_first}
                    
        return False
        
    def _check_subsequent_nodes(self, parent):
        # Check all directions
        found_goal = False
        for branch_info in self.action_set.values(): 
            pos = self.get_next_pos(parent[1], branch_info['move'])
            found_goal = self._get_next_branch(parent, pos, branch_info['move'], branch_info['cost']) or found_goal
        
        # If any of these are true, we have reached the goal state. Propogate the true to the next level
        return found_goal
    
    # Find path
    def find_path(self):
        self._find_path()
        
#<=============================== AStarSearch Class Definition ===================================>#
class AStarSearch(Search):
    
    """
    A* Search Class
    Parent: Search
    
    arg: map, action set dict, scale percent, add frame frequency, framerate
    
    Executes a A* search on the given map with the given action set.
    """    
    
    def __init__(self, map, action_set, cost_to_follow_weight=1, scale_percent=100, add_frame_frequency=100, framerate=100, save_video=True):
        super().__init__(map, action_set, cost_algorithm=True, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate, save_video=save_video)
        self.weight = cost_to_follow_weight

    # Grabs next node
    def _get_next_branch(self, parent, pos, action, branch_cost): 
         
        # Get the new position using tuple addition  
        parent = parent[1] 
        
        # Check if new position is on the grid
        on_grid =             pos[1] < self.height
        on_grid = on_grid and pos[1] >= 0
        on_grid = on_grid and pos[0] < self.width
        on_grid = on_grid and pos[0] >= 0 
        
        # Check if move is legal and generate that new state
        if (on_grid):
            loc_string = Search.pos2str(pos) 
            theta = 0
            if not self.discrete:
                theta = pos[2]
            
            parent_loc_string = Search.pos2str(parent)
            
            # Calculate costs
            try:
                parent_cost = self.node_info[parent_loc_string][parent[2]]["cost_from_first"]
            except KeyError:
                return False
                    
            cost_from_first = parent_cost+branch_cost
            total_cost = cost_from_first + self.weight*math.dist(self.goal_pos[:2], pos[:2])
            # math.sqrt((self.goal_pos[0]-pos[0])**2 + (self.goal_pos[1]-pos[1])**2)
            node_info = self.node_info.get(loc_string, None)
            
            # If position corresponds to the winning state, return True
            if self.at_goal_pos(pos):
                if node_info == None:
                    self.node_info[loc_string] = {}
                self.node_info[loc_string][theta] = {"parent": parent, "action": action, "cost_from_first": cost_from_first}
                self.winning_loc = pos
                return True 
        
            # Check if the node is on an obstacle
            if self.no_obstacles(pos):
                
                # Check if the node has not been visited 
                if self.not_visited(node_info, theta):
                    
                    # Change pixel color to blue unless it overlaps with the start and end depictions
                    self.draw_on_grid(parent, pos, (255,0,0))
                    
                    # Update queue and parent info
                    if node_info == None:
                        self.node_info[loc_string] = {}
                    self.node_info[loc_string][theta] = {"parent": parent, "action": action, "cost_from_first": cost_from_first}
                    self.q_set(total_cost, pos)
                    return False
                
                # Replace parent node if smaller cost path can be found
                keys = list(node_info.keys())
                closest_theta = keys[np.argmin([not self.angle_thresh(theta, theta_i) for theta_i in keys])]
                if cost_from_first < node_info[closest_theta]["cost_from_first"] and parent != pos:
                    self.node_info[loc_string].pop(closest_theta)
                    self.node_info[loc_string][theta] = {"parent": parent, "action": action, "cost_from_first": cost_from_first}
                    
        return False
        
    def _check_subsequent_nodes(self, parent):
        # Check all directions
        found_goal = False
        for branch_info in self.action_set.values():            
            # print("==============")
            # print("parent", parent)
            # print("move", branch_info['move'])
            pos = self.get_next_pos(parent[1], branch_info['move'])
            found_goal = self._get_next_branch(parent, pos, branch_info['move'], branch_info['cost']) or found_goal
        #     print("pos", pos)
        #     print("==============")
        #     self.show_grid(self.grid, wait=1)
        # print("grid shape", self.grid.shape)
        
        # exit()
        # If any of these are true, we have reached the goal state. Propogate the true to the next level
        return found_goal
    
    # Find path
    def find_path(self):
        self._find_path()

#<=============================== RRTStarSearch Class Definition ===================================>#
class RRTStarSearch(Search):
    
    """
    RRT* Search Class
    Parent: Search
    
    arg: map, action set dict, scale percent, add frame frequency, framerate
    
    Executes a RRT* search on the given map with the given action set.
    """    
    
    def __init__(self, map, action_set, max_iters=1250, check_cost_radius=40, scale_percent=100, add_frame_frequency=100, framerate=100, save_video=True):
        super().__init__(map, action_set, cost_algorithm=True, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate, save_video=save_video)
        self.check_cost_radius = check_cost_radius
        self.max_iters = max_iters

    # Grabs next node for RRT* implementation
    def _get_next_branch_rrt(self, parent, pos, action, branch_cost): 
         
        # Check if new position is on the grid
        on_grid =             pos[1] < self.height
        on_grid = on_grid and pos[1] >= 0
        on_grid = on_grid and pos[0] < self.width
        on_grid = on_grid and pos[0] >= 0 
        
        # Check if move is legal and generate that new state
        if (on_grid):
            loc_string = Search.pos2str(pos) 
            theta = 0
            if not self.discrete:
                theta = pos[2]
            
            parent_loc_string = Search.pos2str(parent)
            
            # Calculate costs
            try:
                parent_cost = self.node_info[parent_loc_string][parent[2]]["cost_from_first"]
            except KeyError:
                return
                    
            cost_from_first = parent_cost+branch_cost
            node_info = self.node_info.get(loc_string, None)
        
            # Check if the node is on an obstacle
            if self.no_obstacles(pos):
                
                # Check if the node has not been visited 
                if self.not_visited(node_info, theta):
                    
                    # Change pixel color to blue unless it overlaps with the start and end depictions
                    self.draw_on_grid(parent, pos, (255,0,0))
                    
                    # Update queue and parent info
                    if node_info == None:
                        self.node_info[loc_string] = {}
                    self.node_info[loc_string][theta] = {"parent": parent, "action": action, "cost_from_first": cost_from_first}
                    self.node_count += 1
                    return
                
                # Replace parent node if smaller cost path can be found
                keys = list(node_info.keys())
                closest_theta = keys[np.argmin([not self.angle_thresh(theta, theta_i) for theta_i in keys])]
                if cost_from_first < node_info[closest_theta]["cost_from_first"] and parent != pos:
                    self.node_info[loc_string].pop(closest_theta)
                    self.node_info[loc_string][theta] = {"parent": parent, "action": action, "cost_from_first": cost_from_first}
                    self.node_count += 1
        
    # Select next node 
    def build_next_node(self):
        # Choose random pos
        rand_pos = (np.random.randint(self.width), np.random.randint(self.height))

        # Find all visited nodes within the given radius from random state
        keys = list(self.node_info.keys())
        int_ = lambda ls: [int(val) for val in ls]
        all_dists = np.array([math.dist(rand_pos, int_(pos_strs.split(" "))) for pos_strs in keys])
        dists_in_rad = np.where(all_dists < self.check_cost_radius)[0]
        
        # Select parent node
        best_parent_key = None
        best_theta = None
        # if no visited nodes within radius...
        if len(dists_in_rad) == 0: 
            # select the closest node
            best_parent_key = keys[np.argmin(all_dists)]
            items = list(self.node_info[best_parent_key].items())
            cost_ls = [value["cost_from_first"] for _, value in items]
            min_theta_cost_index = np.argmin(cost_ls)
            best_theta = items[min_theta_cost_index][0]
        else:
            # Choose node with smallest cost to come
            keys_in_rad = [keys[index] for index in dists_in_rad]
            costs = []
            for key in keys_in_rad:
                items = list(self.node_info[key].items())
                cost_ls = [value["cost_from_first"] for _, value in items]
                min_theta_cost_index = np.argmin(cost_ls)
                heapq.heappush(costs, (cost_ls[min_theta_cost_index], (key, items[min_theta_cost_index][0])))
            best_parent_key = costs[0][1][0]
            best_theta = costs[0][1][1]

        best_parent = int_(best_parent_key.split(" "))
        best_parent.append(best_theta)
        
        # Find best action for that given parent
        action_set_moves = []
        action_set_vals = list(self.action_set.values())
        for branch_info in action_set_vals:            
            action_set_moves.append(self.get_next_pos(best_parent, branch_info['move']))

        arg_min = np.argmin([math.dist(rand_pos, moved[:2]) for moved in action_set_moves])
        best_move = action_set_moves[arg_min]

        # Try node
        self._get_next_branch_rrt(best_parent, best_move, branch_info['move'], branch_info['cost'])
    
    # A* implementation post RRT*
    def _get_next_branch(self, parent, pos, action, branch_cost): 
         
        # Get the new position using tuple addition  
        parent = parent[1] 
        
        loc_string = Search.pos2str(pos) 
        theta = 0
        if not self.discrete:
            theta = pos[2]
        
        parent_loc_string = Search.pos2str(parent)
        
        # Calculate costs
        try:
            parent_cost = self.node_info[parent_loc_string][parent[2]]["cost_from_first"]
        except KeyError:
            return False
                
        cost_from_first = parent_cost+branch_cost
        total_cost = cost_from_first + 2*math.dist(self.goal_pos[:2], pos[:2])
        # math.sqrt((self.goal_pos[0]-pos[0])**2 + (self.goal_pos[1]-pos[1])**2)
        node_info = self.node_info.get(loc_string, None)
        
        # If position corresponds to the winning state, return True
        if self.at_goal_pos(pos):
            if node_info == None:
                self.node_info[loc_string] = {}
            self.node_info[loc_string][theta] = {"parent": parent, "action": action, "cost_from_first": cost_from_first}
            self.winning_loc = pos
            print("Found goal pos")
            return True 

        # Check if the node has not been visited 
        if self.not_visited(node_info, theta):
            
            # Change pixel color to blue unless it overlaps with the start and end depictions
            self.draw_on_grid(parent, pos, (255,255,0))
            
            # Update queue and parent info
            if node_info == None:
                self.node_info[loc_string] = {}
            self.node_info[loc_string][theta] = {"parent": parent, "action": action, "cost_from_first": cost_from_first}
            self.q_set(total_cost, pos)
            return False
        
        # Replace parent node if smaller cost path can be found
        keys = list(node_info.keys())
        closest_theta = keys[np.argmin([not self.angle_thresh(theta, theta_i) for theta_i in keys])]
        if cost_from_first < node_info[closest_theta]["cost_from_first"] and parent != pos:
            self.node_info[loc_string].pop(closest_theta)
            self.node_info[loc_string][theta] = {"parent": parent, "action": action, "cost_from_first": cost_from_first}
                    
        return False

    # Get next node
    def _check_subsequent_nodes(self, parent):
        # For every action that results in an equivalent RRT* node...
        found_goal = False
        for branch_info in self.action_set.values():            
            pos = self.get_next_pos(parent[1], branch_info['move'])
            pos_str = self.pos2str(pos)
            if self.rrt_node_info.get(pos_str, None) is not None:
                # Try the node
                found_goal = self._get_next_branch(parent, pos, branch_info['move'], branch_info['cost']) or found_goal

        # self.show_grid(self.grid, scale_percent=300, wait=1, start_pos=self.start_pos, goal_pos=self.goal_pos)
        
        # If any of these are true, we have reached the goal state. Propogate the true to the next level
        return found_goal
    

    # Find path
    def find_path(self):
        # Generate RRT* graph
        print("\nGenerating RRT* Graph...")
        start_time = time()

        self.node_count = 0
        count = 0
        while (self.node_count < self.max_iters):            
            self.build_next_node()
            
            # Only add frame to video at specified intervals
            count += 1
            if count == self.add_frame_frequency:
                self.frames.append(self.grid.copy())
                # self.show_grid(self.grid, wait=0)
                count = 0

        self.rrt_node_info = self.node_info.copy()
        self.node_info.clear()
        orig_parent_loc_string = Search.pos2str(self.start_pos)
        self.node_info = {orig_parent_loc_string : {self.start_pos[2] : {"cost_from_first" : 0}}}

        # Apply A* on graph
        self._find_path(start_time)