import numpy as np
import operator
import os
import cv2
import queue
import time
import math
import matplotlib.pyplot as plt

class BFSSearch:
    def __init__(self, GridClass, scale_percent=100):
        self.grid, self.height, self.width = GridClass.gen_grid()
        
        while True:
            start_str = input(f"Enter start position as x,y (0 <= x <= {self.width-1} ',' and 0 <= y <= {self.height-1}): ")
            start_str_list = start_str.replace(" ", "").split(",")
            try:
                x_pos = int(start_str_list[0])
                y_pos = self.height-1-int(start_str_list[1])
            except:
                print("Please add a comma between the two numbers.\n")
                continue    
            if x_pos < 0 or x_pos >= self.width or y_pos < 0 or y_pos >= self.height:
                print("Numbers out of bounds. Please select new position.\n")
                continue
            if np.all(self.grid[y_pos, x_pos] == (0,0,255)):
                print("There is an obstacle here. Please select a new position.\n")
                continue
            self.start_pos = (x_pos, y_pos)
            break
        
        print("")
        
        while True:
            goal_str = input(f"Enter goal position as x,y (0 <= x <= {self.width-1} ',' and 0 <= y <= {self.height-1}): ")
            goal_str_list = goal_str.replace(" ", "").split(",")
            try:
                x_pos = int(goal_str_list[0])
                y_pos = self.height-1-int(goal_str_list[1])
            except:
                print("Please add a comma between the two numbers.\n")
                continue    
            if x_pos < 0 or x_pos >= self.width or y_pos < 0 or y_pos >= self.height:
                print("Numbers out of bounds. Please select new position.\n")
                continue
            if np.all(self.grid[y_pos, x_pos] == (0,0,255)):
                print("There is an obstacle here. Please select a new position.\n")
                continue
            self.goal_pos = (x_pos, y_pos)
            break
        
        print("\nSelect image window and press any key to begin.\nHit 'q' at any time to quit execution.")
        
        self.scale_percent = scale_percent
        BFSSearch.show_grid(self.grid, scale_percent, self.start_pos, self.goal_pos)
        
        orig_parent_loc_string = str(self.start_pos[0]) + " " + str(self.start_pos[1])
        self.depth_counter = 0
        self.parent_info = {orig_parent_loc_string : {"count" : self.depth_counter}}
        
        self.current_set = queue.Queue()                                        # Queue of states to inspect 
        self.current_set.put_nowait(self.start_pos)
        self.uint8_0 = np.uint8(0)  
   
    @staticmethod
    def show_grid(grid, scale_percent=100, start=None, end=None, wait=0):
        if start != None and end != None:
            grid = grid.copy()
            cv2.circle(grid, start, 3, (0,255,0), -1)
            cv2.circle(grid, end, 3, (0,255,0), -1)
            
        width = int(grid.shape[1] * scale_percent / 100)
        height = int(grid.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(grid, dim, interpolation = cv2.INTER_AREA) 
        
        cv2.imshow("grid",resized)
        k = cv2.waitKey(wait) & 0xff
        if k == ord('q'):
           exit()
            
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
                    BFSSearch.show_grid(self.grid, self.scale_percent, wait=1)
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
                BFSSearch.show_grid(self.grid, self.scale_percent, self.start_pos, self.goal_pos)
                
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
        grid[40-1: 60, 90-1:110] = (0,0,255)
        center_circ = (width-1-40, 50-1)
        cv2.circle(grid, center_circ, 15, (0,0,255), -1)
        return grid[::-1,:], height, width        
    
class SubmissionMap:
    def __init__(self) -> None:
        pass
    
    @staticmethod
    def gen_grid():
        width = 400
        height = 300
        grid = np.zeros((height, width, 3), np.uint8)
        
        center_circ = (90-1, 70-1)
        cv2.circle(grid, center_circ, 35, (0,0,255), -1)
        
        deg2rad = lambda deg: deg*math.pi/180
        int_ = lambda val: int(round(val))
        
        lower_left_corner = [48-1, 108-1]
        angle = deg2rad(35)
        upper_left_corner = [int_(lower_left_corner[0] + 20*math.cos(angle+deg2rad(90)))-1, int_(lower_left_corner[1] + 20*math.sin(angle+deg2rad(90)))-1]
        lower_right_corner = [int_(lower_left_corner[0] + 150*math.cos(angle))-1, int_(lower_left_corner[1] + 150*math.sin(angle))-1]
        upper_right_corner = [int_(upper_left_corner[0] + 150*math.cos(angle))-1, int_(upper_left_corner[1] + 150*math.sin(angle))-1]        
        pts = np.array([upper_left_corner, lower_left_corner, lower_right_corner, upper_right_corner], np.int32).reshape((-1,1,2))
        cv2.fillPoly(grid, [pts], (0,0,255))
        
        center_ell = (246-1, 145-1)
        cv2.ellipse(grid, center_ell, (120//2, 60//2), 0, 0, 360, (0,0,255), -1)
        
        left_x = 200-1
        top_y = height-1-20
        bot_y = top_y-50
        top_left = [left_x, top_y]
        bot_left = [left_x, bot_y]
        bot_right = [left_x+30, bot_y]
        bot_mid_right = [left_x+30, bot_y+10]
        bot_mid = [left_x+10, bot_y+10]
        top_mid = [left_x+10, bot_y+40]
        top_mid_right = [left_x+30, bot_y+40]
        top_right = [left_x+30, top_y]
        pts = np.array([top_left, bot_left, bot_right, bot_mid_right, bot_mid, top_mid, top_mid_right, top_right], np.int32).reshape((-1,1,2))
        cv2.fillPoly(grid, [pts], (0,0,255))
        
        angle_low = deg2rad(45)
        angle_high = deg2rad(115)
        bot = [width-1-72, 63-1]
        right_x = bot[0] + int_(75*math.cos(angle_low))
        right_bot_y = bot[1] + int_(75*math.sin(angle_low))
        mid_right = [right_x, right_bot_y]
        top_right = [right_x, right_bot_y+55]
        left = [bot[0]+int_(60*math.cos(angle_low+deg2rad(90))), bot[1]+int_(60*math.sin(angle_low+deg2rad(90)))]
        top_mid = [left[0]+int_(60*math.cos(angle_low)), left[1]+int_(60*math.sin(angle_low))]
        bot_mid = [354-1, 138-1]
        pts = np.array([bot, mid_right, top_right, bot_mid, top_mid, left], np.int32).reshape((-1,1,2))
        cv2.fillPoly(grid, [pts], (0,0,255))
            
        return grid[::-1,:], height, width        
    
if __name__ == '__main__':
    os.system('cls')
    trial_map = BFSSearch(SubmissionMap, 100)
    trial_map.find_path()