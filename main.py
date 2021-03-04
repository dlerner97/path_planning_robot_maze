import numpy as np
import operator
import os
import cv2
import queue
import time

class BFSSearch:
    def __init__(self, GridClass):
        self.grid, self.height, self.width = GridClass.gen_grid()
        
        self.start_pos = (10, 50)
        self.goal_pos = (self.width - 20, 50) 
        
        BFSSearch.show_grid(self.grid, 300)
        
        orig_parent_loc_string = str(self.start_pos[0]) + " " + str(self.start_pos[1])
        self.depth_counter = 0
        self.parent_info = {orig_parent_loc_string : {"count" : self.depth_counter}}
        
        self.current_set = queue.Queue()                                        # Queue of states to inspect 
        self.current_set.put_nowait(self.start_pos)
        self.uint8_0 = np.uint8(0)  
   
    @staticmethod
    def show_grid(grid, scale_percent=100, wait=0):
        width = int(grid.shape[1] * scale_percent / 100)
        height = int(grid.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(grid, dim, interpolation = cv2.INTER_AREA) 
        
        cv2.imshow("grid",resized)
        cv2.waitKey(wait)
            
    @staticmethod
    def pos2str(pos):
        return str(pos[0]) + " " + str(pos[1]) 

    def get_next_branch(self, parent, from_dir):
        # Get the new position using tuple addition
        pos = tuple(map(operator.add, parent, (from_dir)))

        on_grid =             pos[1] < self.height
        on_grid = on_grid and pos[1] >= 0
        on_grid = on_grid and pos[0] < self.width
        on_grid = on_grid and pos[0] >= 0 
        
        # Check if move is legal and generate that new state
        if (on_grid):
            loc_string = BFSSearch.pos2str(pos) 
            parent_loc_string = BFSSearch.pos2str(parent)
            
            # If that corresponds to the winning state, return True
            if pos[0] == self.goal_pos[0] and pos[1] == self.goal_pos[1]:
                self.parent_info[loc_string] = {"parent": parent_loc_string, "count": self.depth_counter}
                return True
            
            color = self.grid[pos[1], pos[0]]
            if self.parent_info.get(loc_string) == None and color[-1] == self.uint8_0:
                self.parent_info[loc_string] = {"parent": parent_loc_string, "count": self.depth_counter}
                self.current_set.put_nowait(pos)
        return False
    
    def fill_void(self, parent):
        self.grid[parent[1], parent[0]] = (255,0,0)

        # Check all directions
        found_goal = self.get_next_branch(parent, (1,1))
        found_goal = self.get_next_branch(parent, (1,0))   or found_goal
        found_goal = self.get_next_branch(parent, (1,-1))  or found_goal
        found_goal = self.get_next_branch(parent, (0,1))   or found_goal
        found_goal = self.get_next_branch(parent, (0,-1))  or found_goal
        found_goal = self.get_next_branch(parent, (-1,1))  or found_goal
        found_goal = self.get_next_branch(parent, (-1,0))  or found_goal
        found_goal = self.get_next_branch(parent, (-1,-1)) or found_goal
    
        # If any of these are true, we have reached the goal state. Propogate the true to the next level
        return found_goal
    
    # Search the current current queue and generate all depth levels
    def find_path(self):
        start = time.time()
        self.solution_found = False
        count = 0
        try:
            while (not self.solution_found) :#and (time.time() < start+30):
                # start = time.time()
                self.depth_counter += 1
                if self.current_set.empty():
                    break
                parent = self.current_set.get_nowait()
                self.solution_found = self.fill_void(parent)
                count += 1
                if count == 100:
                    BFSSearch.show_grid(self.grid, 200, 1)
                    count = 0
                
            
            if self.solution_found:
                # Go up the tree to find all parent nodes and optimized path
                next_set = self.parent_info[BFSSearch.pos2str(self.goal_pos)]
                str_list = BFSSearch.pos2str(self.goal_pos).split(" ")
                num_list = [int(str_list[0]), int(str_list[1])]
                path = [num_list]
                try:
                    while True:
                        next_set_ind = next_set['parent']
                        next_set = self.parent_info[next_set_ind]
                        str_list = next_set_ind.split(" ")
                        num_list = [int(str_list[0]), int(str_list[1])]
                        path.append(num_list)
                except KeyError:
                    pass

                # Print full move set
                path.reverse()
                self.path = np.asarray(path)
                self.grid[self.path[:, 1], self.path[:, 0]] = (0,255,0)
                BFSSearch.show_grid(self.grid, 300, 0)
                
        except KeyboardInterrupt:
            print("Keyboard Interrupt")

class TrialMap:
    def __init__(self):
        pass
    
    @staticmethod
    def gen_grid():
        width = 200
        height = 100
        grid = np.zeros((height,width, 3), np.uint8)
        grid[height - 1 - 60: height - 1 - 40, 90:110] = (0,0,255)
        center_circ = (width-1-40, height-1-50)
        cv2.circle(grid, center_circ, 15, (0,0,255), -1)
        return grid, height, width        
    
# class SubmissionMap:
#     def __init__(self) -> None:
#         pass
    
#     @staticmethod
#     def gen_grid():
#         width = 
    
if __name__ == '__main__':
    os.system('cls')
    trial_map = BFSSearch(TrialMap)
    trial_map.find_path()