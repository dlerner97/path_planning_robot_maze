import cv2
import time
import math
import queue
import heapq
import operator
import numpy as np
import matplotlib.pyplot as plt

# This class handles simple image and video tasks
class VideoBuffer():
    def __init__(self, framerate = 100, scale_percent=100):
        self.frames = []
        self.framerate = framerate
        self.scale_percent = scale_percent
      
    # Resizes each length image at a given percent. E.g. 200 will double each dimension and 50 will half it
    @staticmethod
    def resize_image(grid, scale_percent=100):
        width = int(grid.shape[1] * scale_percent / 100)
        height = int(grid.shape[0] * scale_percent / 100)
        dim = (width, height)
        return cv2.resize(grid, dim, interpolation = cv2.INTER_AREA) 
      
    # Displays grid at a given scale
    @staticmethod
    def show_grid(grid, scale_percent=100, wait=0):    
        resized = VideoBuffer.resize_image(grid, scale_percent) 
        # plt.imshow(grid, "gray")
        # plt.show()
        cv2.imshow("grid",resized)
        k = cv2.waitKey(wait) & 0xff
        if k == ord('q'):
           exit()
           
    # Write frames to feed and save video
    def save(self, video_name, isColor = False):
        shape = self.frames[0].shape
        size = (shape[1], shape[0])
        videowriter = cv2.VideoWriter(video_name + ".avi", cv2.VideoWriter_fourcc(*'DIVX'),self.framerate , size, isColor)
        for f in self.frames:
            videowriter.write(f)
        videowriter.release()

# Parent: VideoBuffer
# Child: Actual Search Algorithm
# This class handles the general search implementation. To run this code, however 
class Search(VideoBuffer):
    # Takes in video_name, default start/goal positions, output feed scale factor, number of nodes between subsequent frames, video framerate
    def __init__(self, map, radius=0, clearence=0, cost_algorithm=False, scale_percent=100, add_frame_frequency=100, framerate = 100):
        # Init VideoBuffer Parent
        super().__init__(framerate=framerate, scale_percent=scale_percent)
        print("\n=====================================================================================\n")
        
        # Define map
        self.video_name, self.grid, self.height, self.width = map.gen_grid()
        start_pos_def = (10, self.height//2)
        goal_pos_def = (self.width-2, self.height//2)
        
        # Query user for positions
        def get_user_input(pos_string, default):            
            # Query until user has input legal values     
            while True:
                start_str = input(f"Enter {pos_string} position as x,y (0 <= x <= {self.width-1} ',' and 0 <= y <= {self.height-1}) or leave blank to apply the default values: ")
                start_str_nw = start_str.replace(" ", "")
                
                # If empty string, set values to default
                if start_str_nw == '':
                    print("Selecting default values")
                    return default              
                    
                start_str_list = start_str_nw.split(",")
                
                # Check for incorrect input
                try:
                    x_pos = int(start_str_list[0])
                    y_pos = self.height-1-int(start_str_list[1])
                except:
                    print("Please type two integers with a comma in between.\n")
                    continue    
                
                # Check if position out of bounds
                if x_pos < 0 or x_pos >= self.width or y_pos < 0 or y_pos >= self.height:
                    print("Numbers out of bounds. Please select new position.\n")
                    continue
                if np.all(self.grid[y_pos, x_pos] == (0,0,255)):
                    print("There is an obstacle here. Please select a new position.\n")
                    continue
                
                # User chose correct inputs
                return (x_pos, y_pos)
        
        # Define start/goal positions
        self.start_pos = get_user_input("start", start_pos_def)
        print("")
        self.goal_pos = get_user_input("goal", goal_pos_def)
        print("\n=====================================================================================")
        
        print("\nSelect image window and press any key to begin. Regardless, code will run in 10 seconds\nHit 'ctrl+C' at any time to quit execution.")
        
        # Add start/goal positions to grid
        temp_grid = self.grid.copy()
        cv2.circle(temp_grid, self.start_pos, 3, (0,255,0), -1)
        cv2.circle(temp_grid, self.goal_pos, 3, (0,255,0), -1)
        self.grid = temp_grid
        
        # Display grid
        self.scale_percent = scale_percent
        Search.show_grid(self.grid, scale_percent, wait=10000)
        cv2.destroyAllWindows()
        self.frames.append(self.grid)
        
        # Define variables
        #                                                 
        orig_parent_loc_string = str(self.start_pos[0]) + " " + str(self.start_pos[1])
        if cost_algorithm:
            self.node_info = {orig_parent_loc_string : {"cost_from_first": 0}}          # Dictionary of node info
            self.current_set = [(0, self.start_pos)]
            heapq.heapify(self.current_set)
            self.q_set = lambda cost, pos: heapq.heappush(self.current_set, (cost, pos))
            self.q_get = lambda: heapq.heappop(self.current_set)
        else:
            self.node_info = {orig_parent_loc_string : {}}          # Dictionary of node info
            self.current_set = queue.Queue()                            # Queue of states to inspect
            self.q_set = lambda pos: self.current_set.put_nowait(pos)
            self.q_get = lambda: self.current_set.get_nowait()
            self.q_set(self.start_pos)
                                                           
        self.add_frame_frequency = round(add_frame_frequency/8)        
        self.uint8_0 = np.uint8(0)                                                       # Define a 0 as an 8-bit unsigned int. This will save processing time    
        self.radius = radius
        self.clearence = clearence 
        self.obstacle_dict = {}
          
    # Convert position tuple into string for dictionary storage
    @staticmethod
    def pos2str(pos):
        return str(pos[0]) + " " + str(pos[1]) 
    
    def no_obstacles(self, curr_pos):
        if self.grid[curr_pos[1], curr_pos[0]][-1] != self.uint8_0:
            return False
        if self.radius + self.clearence == 0:
            return True
            
        r = self.radius + self.clearence
        
        from_x = max(0, curr_pos[0]-r)
        to_x = min(self.width-1, curr_pos[0]+r)
        from_y = max(0, curr_pos[1]-r)
        to_y = min(self.height-1, curr_pos[1]+r)
        
        wall_indeces = np.where(self.grid[from_y:to_y, from_x:to_x, -1] != self.uint8_0)
        # print(wall_indeces)
        for x, y in zip(wall_indeces[0]-r, wall_indeces[1]-r):
            if np.linalg.norm([x,y]) < r:
                return False
        return True
        
    # Protected method for searching the current current queue and generate all depth levels. 
    # This method and the _find_path method are protected since they can not be run alone. This is a general method and does not implement any specific search algorithm.
    # Rather, it should be absorbed by an actual search algorithm (BFS, Dijkstra, etc.). True search algorithm MUST have a method called "_check_subsequent nodes"
    #   and store nodes to the proper queue  
    def _find_path(self):
        print("\nFinding Path...")
        start_time = time.time()                                # Time path planning
        solution_found = False
        count = 0
        c1 = 0
        try:
            while (not solution_found):

                # Pull state from queue and check subsequent nodes
                try:
                    parent = self.q_get()
                except IndexError:
                    break
                except queue.Empty:
                    break
                
                solution_found = self._check_subsequent_nodes(parent)
                
                # Only add frame to video at specified intervals
                count += 1
                c1 += 1
                if count == self.add_frame_frequency:
                    self.frames.append(self.grid.copy())
                    count = 0
                # if c1 == 20:
                #     self.show_grid(self.grid, 300)
                #     exit()
            
            print("\n=====================================================================================")
            if solution_found:
                # Go up the dictionary tree to find all parent nodes and optimized path
                next_set = self.node_info[Search.pos2str(self.goal_pos)]
                str_list = Search.pos2str(self.goal_pos).split(" ")
                num_list = [int(str_list[0]), int(str_list[1])]
                path = [num_list]
                try:
                    while True:
                        # print('-'*10)
                        next_set_ind = next_set['parent']
                        next_set = self.node_info[next_set_ind]
                        
                        # Parse strings and add them to path
                        str_list = next_set_ind.split(" ")
                        num_list = (int(str_list[0]), int(str_list[1]))
                        path.append(num_list)
                            
                        # Change grid color and add frames to video
                        self.grid[num_list[1], num_list[0]] = (0,255,0)
                        self.frames.append(self.grid.copy())
                        # self.show_grid(self.grid, 300)
                        # print('-'*10)
                except KeyError:
                    pass

                # Print full move set
                path.reverse()
                self.path = np.asarray(path)
                self.grid[self.path[:, 1], self.path[:, 0]] = (0,255,0)
                
                # Calculate time elapsed
                time_elapsed_s = time.time() - start_time
                time_elapsed_mins = time_elapsed_s//60
                time_elapsed_hrs = time_elapsed_s//60**2
                time_elapsed_secs = time_elapsed_s%60
                
                print(f"\nThis path can be solved in {len(self.path)-1} moves")
                print(f"The {self.__class__.__name__} algorithm took {time_elapsed_hrs} hrs, {time_elapsed_mins} mins, and {time_elapsed_secs} s to implement.\n")
              
            # If no solution was found
            else:
                print("\nThe {self.__class__.__name__} algorithm cannot detect a path. Please select new start and goal points or remove obstacles from the grid.")
               
            # Save and write video feed
            self.save(self.video_name + "_animation", True)
            print(f"The video file can be found in your current directory under {self.video_name}_animation.avi")
            print("\n=====================================================================================\n")

            # Display grid
            Search.show_grid(self.grid, self.scale_percent, wait=10000)   
               
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            self.save(self.video_name + "_animation_INTERRUPT", True)

# The BFSSearch class is a wrapper for search.            
class BFSSearch(Search):
    def __init__(self, map, radius, clearence, scale_percent, add_frame_frequency, framerate):
        super().__init__(map, radius=radius, clearence=clearence, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate)
        self.action_set = {
            "up"         :  (0,-1),
            "up_right"   :  (1,-1),
            "right"      :   (1,0),
            "down_right" :   (1,1),
            "down"       :   (0,1),
            "down_left"  :  (-1,1),
            "left"       :  (-1,0),
            "up_left"    : (-1,-1)
        }
    
    def _get_next_branch(self, parent, from_dir):  
        # Get the new position using tuple addition
        pos = tuple(map(operator.add, parent, (from_dir)))

        # Check if new position is on the grid
        on_grid =             pos[1] < self.height
        on_grid = on_grid and pos[1] >= 0
        on_grid = on_grid and pos[0] < self.width
        on_grid = on_grid and pos[0] >= 0 
        
        # Check if move is legal and generate that new state
        if (on_grid):
            loc_string = Search.pos2str(pos) 
            parent_loc_string = Search.pos2str(parent)
            
            # If position corresponds to the winning state, return True
            if pos[0] == self.goal_pos[0] and pos[1] == self.goal_pos[1]:
                self.node_info[loc_string] = {"parent": parent_loc_string}
                return True
            
            # Check if the node has already been visited or if it is on an obstacle
            color = self.grid[pos[1], pos[0]]
            if self.node_info.get(loc_string) == None and color[-1] == self.uint8_0:
                
                # Change pixel color to blue unless it overlaps with the start and end depictions
                if color[1] == self.uint8_0:
                    self.grid[pos[1], pos[0]] = (255,0,0)
                
                # Update queue and parent info
                self.node_info[loc_string] = {"parent": parent_loc_string}
                self.q_set(pos)
        return False
    
    # Check all subsequent directions for a given position
    def _check_subsequent_nodes(self, parent):

        # Check all directions
        found_goal = False
        for dir in self.action_set.values(): 
            found_goal = self._get_next_branch(parent, dir) or found_goal
        
        # If any of these are true, we have reached the goal state. Propogate the true to the next level
        return found_goal
    
    def find_path(self):
        self._find_path()

class DijkstraSearch(Search):
    def __init__(self, map, radius, clearence, scale_percent, add_frame_frequency, framerate):
        super().__init__(map, radius=radius, clearence=clearence, cost_algorithm=True, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate)
        self.action_set = {
            "up"         : {"move":  (0,-1), "cost": 1},
            "up_right"   : {"move":  (1,-1), "cost": math.sqrt(2)},
            "right"      : {"move":   (1,0), "cost": 1},
            "down_right" : {"move":   (1,1), "cost": math.sqrt(2)},
            "down"       : {"move":   (0,1), "cost": 1},
            "down_left"  : {"move":  (-1,1), "cost": math.sqrt(2)},
            "left"       : {"move":  (-1,0), "cost": 1},
            "up_left"    : {"move": (-1,-1), "cost": math.sqrt(2)},
        }
    
    def _get_next_branch(self, parent, from_dir, branch_cost):  
        # Get the new position using tuple addition
        parent = parent[1]
        pos = tuple(map(operator.add, parent, (from_dir)))
        
        # Check if new position is on the grid
        on_grid =             pos[1] < self.height
        on_grid = on_grid and pos[1] >= 0
        on_grid = on_grid and pos[0] < self.width
        on_grid = on_grid and pos[0] >= 0 
        
        # Check if move is legal and generate that new state
        if (on_grid):
            loc_string = Search.pos2str(pos) 
            parent_loc_string = Search.pos2str(parent)
            parent_cost = self.node_info[parent_loc_string]["cost_from_first"]
            cost_from_first = parent_cost+branch_cost
            
            # If position corresponds to the winning state, return True
            if pos[0] == self.goal_pos[0] and pos[1] == self.goal_pos[1]:
                self.node_info[loc_string] = {"parent": parent_loc_string, "cost_from_first": cost_from_first}
                return True 
                       
            # Check if the node is on an obstacle
            color = self.grid[pos[1], pos[0]]
            if color[-1] == self.uint8_0:
                # Check if the node has already been visited
                node_info = self.node_info.get(loc_string)
                
                if node_info == None:
                    # Change pixel color to blue unless it overlaps with the start and end depictions
                    if color[1] == self.uint8_0:
                        self.grid[pos[1], pos[0]] = (255,0,0)
                    
                    # Update queue and parent info
                    self.node_info[loc_string] = {"parent": parent_loc_string, "cost_from_first": cost_from_first}
                    self.q_set(cost_from_first, pos)
                    # print(self.node_info[loc_string])
                
                elif cost_from_first < node_info["cost_from_first"]:
                    self.node_info[loc_string] = {"parent": parent_loc_string, "cost_from_first": cost_from_first}
                    
        return False
        
    def _check_subsequent_nodes(self, parent):
        # Check all directions
        found_goal = False
        for branch_info in self.action_set.values(): 
            found_goal = self._get_next_branch(parent, branch_info['move'], branch_info['cost']) or found_goal
        
        # If any of these are true, we have reached the goal state. Propogate the true to the next level
        return found_goal
    
    def find_path(self):
        self._find_path()
        
class AStarSearch(Search):
    def __init__(self, map, radius, clearence, scale_percent, add_frame_frequency, framerate):
        super().__init__(map, radius=radius, clearence=clearence, cost_algorithm=True, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate)
        
        self.action_set = {
            "up"         : {"move":  (0,-1), "cost": 1},
            "up_right"   : {"move":  (1,-1), "cost": math.sqrt(2)},
            "right"      : {"move":   (1,0), "cost": 1},
            "down_right" : {"move":   (1,1), "cost": math.sqrt(2)},
            "down"       : {"move":   (0,1), "cost": 1},
            "down_left"  : {"move":  (-1,1), "cost": math.sqrt(2)},
            "left"       : {"move":  (-1,0), "cost": 1},
            "up_left"    : {"move": (-1,-1), "cost": math.sqrt(2)},
        }
        
    def _get_next_branch(self, parent, from_dir, branch_cost):  
        # Get the new position using tuple addition    
        parent = parent[1] 
        pos = tuple(map(operator.add, parent, (from_dir)))
        
        # Check if new position is on the grid
        on_grid =             pos[1] < self.height
        on_grid = on_grid and pos[1] >= 0
        on_grid = on_grid and pos[0] < self.width
        on_grid = on_grid and pos[0] >= 0 
        
        # Check if move is legal and generate that new state
        if (on_grid):
            loc_string = Search.pos2str(pos) 
            parent_loc_string = Search.pos2str(parent)
            parent_cost = self.node_info[parent_loc_string]["cost_from_first"]
            cost_from_first = parent_cost+branch_cost
            total_cost = cost_from_first + math.sqrt((self.goal_pos[0]-pos[0])**2 + (self.goal_pos[1]-pos[1])**2)
            
            # If position corresponds to the winning state, return True
            if pos[0] == self.goal_pos[0] and pos[1] == self.goal_pos[1]:
                self.node_info[loc_string] = {"parent": parent_loc_string, "cost_from_first": cost_from_first}
                return True 
                       
            # Check if the node is on an obstacle
            color = self.grid[pos[1], pos[0]]
            # if color[-1] == self.uint8_0:
            if self.no_obstacles(pos):
                # Check if the node has already been visited
                node_info = self.node_info.get(loc_string)
                
                if node_info == None:
                    # Change pixel color to blue unless it overlaps with the start and end depictions
                    if color[1] == self.uint8_0:
                        self.grid[pos[1], pos[0]] = (255,0,0)
                    
                    # Update queue and parent info
                    self.node_info[loc_string] = {"parent": parent_loc_string, "cost_from_first": cost_from_first}
                    self.q_set(total_cost, pos)
                    # print(self.node_info[loc_string])
                
                elif cost_from_first < node_info["cost_from_first"]:
                    self.node_info[loc_string] = {"parent": parent_loc_string, "cost_from_first": cost_from_first}
                    
        return False
        
    def _check_subsequent_nodes(self, parent):
        # Check all directions
        found_goal = False
        for branch_info in self.action_set.values(): 
            found_goal = self._get_next_branch(parent, branch_info['move'], branch_info['cost']) or found_goal
        
        # If any of these are true, we have reached the goal state. Propogate the true to the next level
        return found_goal
    
    def find_path(self):
        self._find_path()