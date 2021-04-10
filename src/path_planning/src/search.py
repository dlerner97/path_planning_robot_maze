#<=============================== Imports ===================================>#
import math
import numpy as np
from search_base import Search

#<=============================== BFSSearch Class Definition ===================================>#
class BFSSearch(Search):
    
    """
    Breadth-First Search Class
    Parent: Search
    
    arg: map, action set dict, scale percent, add frame frequency, framerate
    
    Executes a breadth first searchon the given map with the given action set.
    """
    
    def __init__(self, map, action_set, scale_percent=100, add_frame_frequency=100, framerate=100):
        super().__init__(map, action_set, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate)

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
            found_goal = self._get_next_branch(parent, pos, action) or found_goal
        
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
    
    def __init__(self, map, action_set, scale_percent=100, add_frame_frequency=100, framerate=100):
        super().__init__(map, action_set, cost_algorithm=True, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate)

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
    
    def __init__(self, map, action_set, scale_percent=100, add_frame_frequency=100, framerate=100):
        super().__init__(map, action_set, cost_algorithm=True, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate)

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
            total_cost = cost_from_first + math.sqrt((self.goal_pos[0]-pos[0])**2 + (self.goal_pos[1]-pos[1])**2)
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