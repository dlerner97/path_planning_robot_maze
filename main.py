import os
import cv2
import time
import math
import queue
import operator
import numpy as np

import matplotlib.pyplot as plt

class VideoBuffer():
    def __init__(self, framerate = 100, scale_percent=100):
        self.frames = []
        self.framerate = framerate
        self.scale_percent = scale_percent
      
    @staticmethod
    def resize_image(grid, scale_percent=100):
        width = int(grid.shape[1] * scale_percent / 100)
        height = int(grid.shape[0] * scale_percent / 100)
        dim = (width, height)
        return cv2.resize(grid, dim, interpolation = cv2.INTER_AREA) 
        
    @staticmethod
    def show_grid(grid, scale_percent=100, start=None, end=None, wait=0):
        if start != None and end != None:
            grid = grid.copy()
            cv2.circle(grid, start, 3, (0,255,0), -1)
            cv2.circle(grid, end, 3, (0,255,0), -1)
            
        resized = VideoBuffer.resize_image(grid, scale_percent)
        
        cv2.imshow("grid",resized)
        k = cv2.waitKey(wait) & 0xff
        if k == ord('q'):
           exit()
           
    # filename MUST be '.AVI' type
    # filter_function is a callback that takes each image in and outputs a new filtered image
    # Videowriter DIVX is for windows. Probably won't work on MAC or Windows
    def save(self, video_name, isColor = False):
        shape = self.frames[0].shape
        size = (shape[1], shape[0])
        videowriter = cv2.VideoWriter(video_name + ".avi", cv2.VideoWriter_fourcc(*'DIVX'),self.framerate , size, isColor)
        for f in self.frames:
            videowriter.write(f)
        videowriter.release()

class BFSSearch(VideoBuffer):
    def __init__(self, vid_name, start_pos_def, goal_pos_def, scale_percent=100, add_frame_frequency=100, framerate = 100):
        super().__init__(framerate=framerate, scale_percent=scale_percent)
        
        def get_user_input(pos_string, default):     
            while True:
                start_str = input(f"Enter {pos_string} position as x,y (0 <= x <= {self.width-1} ',' and 0 <= y <= {self.height-1}) or just enter for default: ")
                start_str_nw = start_str.replace(" ", "")
                if start_str_nw == '':
                    print("Selecting default values")
                    return default              
                    
                start_str_list = start_str_nw.split(",")
                try:
                    x_pos = int(start_str_list[0])
                    y_pos = self.height-1-int(start_str_list[1])
                except:
                    print("Please type two integers with a comma in between.\n")
                    continue    
                if x_pos < 0 or x_pos >= self.width or y_pos < 0 or y_pos >= self.height:
                    print("Numbers out of bounds. Please select new position.\n")
                    continue
                if np.all(self.grid[y_pos, x_pos] == (0,0,255)):
                    print("There is an obstacle here. Please select a new position.\n")
                    continue
                return (x_pos, y_pos)
        
        self.start_pos = get_user_input("start", start_pos_def)
        print("")
        self.goal_pos = get_user_input("goal", goal_pos_def)
        
        print("\nSelect image window and press any key to begin.\nHit 'ctrl+C' at any time to quit execution.")
        
        temp_grid = self.grid.copy()
        cv2.circle(temp_grid, self.start_pos, 3, (0,255,0), -1)
        cv2.circle(temp_grid, self.goal_pos, 3, (0,255,0), -1)
        self.grid = temp_grid
        
        self.scale_percent = scale_percent
        BFSSearch.show_grid(self.grid, scale_percent)
        cv2.destroyAllWindows()
        self.frames.append(self.grid)
        
        orig_parent_loc_string = str(self.start_pos[0]) + " " + str(self.start_pos[1])
        self.depth_counter = 0
        self.parent_info = {orig_parent_loc_string : {"count" : self.depth_counter}}
        
        self.current_set = queue.Queue()                                        # Queue of states to inspect 
        self.current_set.put_nowait(self.start_pos)
        self.uint8_0 = np.uint8(0)  
        self.add_frame_frequency = round(add_frame_frequency/8)
        
        self.video_name = vid_name
            
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
                if color[1] == self.uint8_0:
                    self.grid[pos[1], pos[0]] = (255,0,0)
                self.parent_info[loc_string] = {"parent": parent_loc_string, "count": self.depth_counter}
                self.current_set.put_nowait(pos)
        return False
    
    def fill_void(self, parent):
        # self.grid[parent[1], parent[0]] = (255,0,0)

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
        print("\nFinding Path...")
        start_time = time.time()
        self.solution_found = False
        count = 0
        try:
            while (not self.solution_found):
                self.depth_counter += 1
                if self.current_set.empty():
                    break
                parent = self.current_set.get_nowait()
                self.solution_found = self.fill_void(parent)
                count += 1
                if count == self.add_frame_frequency:
                    # BFSSearch.show_grid(self.grid, self.scale_percent, start=self.start_pos, end=self.goal_pos, wait=1)
                    self.frames.append(self.grid.copy())
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
                        self.grid[num_list[1], num_list[0]] = (0,255,0)
                        self.frames.append(self.grid.copy())
                        path.append(num_list)
                except KeyError:
                    pass

                # Print full move set
                path.reverse()
                self.path = np.asarray(path)
                self.grid[self.path[:, 1], self.path[:, 0]] = (0,255,0)
                
                time_elapsed_s = time.time() - start_time
                time_elapsed_mins = time_elapsed_s//60
                time_elapsed_hrs = time_elapsed_s//60**2
                time_elapsed_secs = time_elapsed_s%60
                
                print(f"\nThis path can be solved in {len(self.path)-1} moves")
                print(f"The BFS algorithm took {time_elapsed_hrs} hrs, {time_elapsed_mins} mins, and {time_elapsed_secs} s to implement.\n")
                
                self.save(self.video_name + "_animation", True)
                BFSSearch.show_grid(self.grid, self.scale_percent, self.start_pos, self.goal_pos)
                
            else:
                print("\nThe BFS algorithm cannot detect a path. Please select new start and goal points or remove obstacles from the grid.")
                self.save(self.video_name + "_animation", True)
                BFSSearch.show_grid(self.grid, self.scale_percent, self.start_pos, self.goal_pos)
                
            print(f"The video file can be found in your current directory under {self.video_name}_animation.avi")
        except KeyboardInterrupt:
            print("Keyboard Interrupt")

class TrialMap(BFSSearch):
    def __init__(self, scale_percent=100, add_frame_frequency=100, framerate=100):
        self.grid, self.height, self.width = TrialMap.gen_grid()
        super().__init__("trial_map", (15, 50), (185, 50), scale_percent, add_frame_frequency, framerate)
    
    @staticmethod
    def gen_grid():
        width = 200
        height = 100
        
        grid = np.zeros((height,width, 3), np.uint8)
        grid[40-1: 60, 90-1:110] = (0,0,255)
        center_circ = (width-1-40, 50-1)
        cv2.circle(grid, center_circ, 15, (0,0,255), -1)
        return grid[::-1,:], height, width        
    
class SubmissionMap(BFSSearch):
    def __init__(self, scale_percent=100, add_frame_frequency=100, framerate=100) -> None:
        self.grid, self.height, self.width = SubmissionMap.gen_grid()
        super().__init__("submission_map", (15, 150), (385, 150), scale_percent, add_frame_frequency, framerate)
    
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
    
class TestNoPathMap(BFSSearch):
    def __init__(self, scale_percent=100, add_frame_frequency=100, framerate=100):
        self.grid, self.height, self.width = TestNoPathMap.gen_grid()
        super().__init__("test_no_possible_path", (50, 50), (150, 50), scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate)
    
    @staticmethod
    def gen_grid():
        width = 200
        height = 100
        
        grid = np.zeros((height,width, 3), np.uint8)
        grid[:, 90:100] = (0,0,255)
        
        return grid[::-1,:], height, width
        
        
        
if __name__ == '__main__':
    os.system('cls')

    # trial_map = TrialMap(scale_percent=300,add_frame_frequency=50, framerate=100)
    # trial_map.find_path()
    
    # cv2.destroyAllWindows()

    # submission_map = SubmissionMap(scale_percent=200, add_frame_frequency=300, framerate=100)
    # submission_map.find_path()
    
    # cv2.destroyAllWindows()
    
    test_no_path_map = TestNoPathMap(scale_percent=300, add_frame_frequency=50, framerate=100)
    test_no_path_map.find_path()