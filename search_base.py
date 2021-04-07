#<=============================== Imports ===================================>#
import cv2
import time
import math
import queue
import heapq
import numpy as np

#<=============================== VideoBuffer Class Definition ===================================>#
class VideoBuffer:
    
    """
    Video Buffer Class
    
    Args: output video framerate, grid scaling factor
    
    This class handles video writing and other simple video tasks.
    """
    
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
            cv2.circle(f, self.start_pos[:2], 5, (0,255,0), -1)
            cv2.circle(f, self.goal_pos[:2], 5, (0,255,0), -1)
            videowriter.write(f)
        videowriter.release()
 
 #<=============================== Search Class Definition ===================================>#
class Search(VideoBuffer):
    
    """
    Search Class
    
    Parent: VideoBuffer
    Child: Actual Search Algorithm
    This class handles the general search implementations. It cannot, however, be run alone. It must be the parent of another class
    """

    # Takes in video_name, default start/goal positions, output feed scale factor, number of nodes between subsequent frames, video framerate
    def __init__(self, map_, action_set, cost_algorithm=False, scale_percent=100, add_frame_frequency=100, framerate = 100):
        # Init VideoBuffer Parent
        super().__init__(framerate=framerate, scale_percent=scale_percent)
        print("\n=====================================================================================\n")
        
        # Define map
        self.video_name, self.grid, self.height, self.width = map_.gen_grid()
        start_pos_def = (10, self.height//2)
        goal_pos_def = (self.width-2, self.height//2)
        
        self.discrete = action_set["discrete"]
        
        # Query user for positions
        def get_pos_user_input(pos_string, default):
            # Query until user has input legal values     
            while True:
                start_str = f"Enter {pos_string} position as x,y (0 <= x <= {self.width-1} ',' 0 <= y <= {self.height-1}" 
                if not self.discrete:
                    start_str += ", 0 <= theta <= 359"
                
                start_str = input(start_str + ") or leave blank to apply the default values: ")
                start_str_nw = start_str.replace(" ", "")
                
                # If empty string, set values to default
                if start_str_nw == '':
                    print("Selecting default values: ", (default[0], default[1], 0))
                    return (default[0], default[1], 0)              
                    
                start_str_list = start_str_nw.split(",")
                
                # Check for incorrect input
                try:
                    x_pos = int(start_str_list[0])
                    y_pos = self.height-1-int(start_str_list[1])
                    theta = None
                    if not self.discrete:
                        theta = int(start_str_list[2])
                    elif len(start_str_list) > 2:
                        raise Exception                    
                except:
                    if self.discrete:
                        print("Please type two integers with a comma in between.\n")
                    else:
                        print("Please type three integers with a comma in between.\n")
                    continue    
                
                # Check if position out of bounds
                bounds = x_pos < 0 or x_pos >= self.width or y_pos < 0 or y_pos >= self.height
                if not self.discrete:
                    bounds = bounds or theta < 0 or theta >= 360 
                if bounds:
                    print("Numbers out of bounds. Please select new position.\n")
                    continue
                if np.all(self.grid[y_pos, x_pos] == (0,0,255)):
                    print("There is an obstacle here. Please select a new position.\n")
                    continue
                
                # User chose correct inputs
                if self.discrete:
                        return (x_pos, y_pos, 0)
                return (x_pos, y_pos, theta)
            
        def get_user_input(pos_string):                       
            # Query until user has input legal values     
            while True:
                start_str = input(f"Enter step size distance (0 <= {pos_string}) or leave blank to apply the default values: ")
                start_str_nw = start_str.replace(" ", "")
                
                # If empty string, set values to default
                if start_str_nw == '':
                    print("Selecting default value: ", 0)
                    return 0              
                                    
                # Check for incorrect input
                try:
                    val = int(start_str_nw)
                except:
                    print("Please type a single integer.\n")
                    continue    
                
                # Check if position out of bounds
                if val < 0:
                    print("Numbers out of bounds. Please select new value.\n")
                    continue

                # User chose correct inputs
                return val
        
        # Define start/goal positions
        self.start_pos = get_pos_user_input("start", start_pos_def)                     # Start position
        print("")
        self.goal_pos = get_pos_user_input("goal", goal_pos_def)                        # Goal position
        print("")
        self.radius = get_user_input("radius")                                          # Robot radius
        print("")
        self.clearence = get_user_input("clearence")                                    # Robot clearence
        
        print("\n=====================================================================================")
        
        print("\nSelect image window and press any key to begin. Regardless, code will run in 10 seconds")
        print("Hit 'ctrl+C' at any time to quit execution.")
        
        # Add start/goal positions to grid
        temp_grid = self.grid.copy()
        cv2.circle(temp_grid, self.start_pos[:2], 3, (0,255,0), -1)
        cv2.circle(temp_grid, self.goal_pos[:2], 3, (0,255,0), -1)
        
        # Display grid
        self.scale_percent = scale_percent
        Search.show_grid(temp_grid, scale_percent, wait=10000)
        cv2.destroyAllWindows()
        scale = 1
        
        # Define variables
        
        orig_parent_loc_string = None
        
        # Differentiate between "discrete" path planning and more realistic robot turn angle/forward proportion distance path planning
        if self.discrete:
            # Define new position from previous and change
            self.get_next_pos = lambda parent, change: (parent[0]+change[0], parent[1]+change[1], 0) 
            
            # Check if at goal position
            self.at_goal_pos = lambda pos: pos[0] == self.goal_pos[0] and pos[1] == self.goal_pos[1]
            self.angle_threshold = 0
        else:
            # Define new position from previous and change
            cos = lambda angle: math.cos(angle*math.pi/180)
            sin = lambda angle: math.sin(angle*math.pi/180)
            add_angs = lambda angle1, angle2: (angle1+angle2+360)%360
            self.get_next_pos = lambda parent, change: (int(round(parent[0]+change[0]*cos(add_angs(change[1],parent[2])))), 
                                                        int(round(parent[1]-change[0]*sin(add_angs(change[1],parent[2])))), add_angs(parent[2],change[1]))
            
            # Scale grid to account for xy distance threshold
            node_threshold = action_set["node_threshold"]
            scale = 1/node_threshold[0]
            self.grid = self.resize_image(self.grid, 100*scale)
            self.goal_pos = (int(scale*self.goal_pos[0]), int(scale*self.goal_pos[1]), self.goal_pos[2])
            self.start_pos = (int(scale*self.start_pos[0]), int(scale*self.start_pos[1]), self.start_pos[2]) 
            self.scale_percent *= node_threshold[0]
            self.angle_threshold = node_threshold[1]
            self.width *= scale
            self.height *= scale
            
            # Check if at goal position
            goal_threshold = action_set['goal_threshold']
            goal_threshold_xy = scale*goal_threshold[0]
            ang = lambda pos: abs(add_angs(pos[2], -self.goal_pos[2])) <= goal_threshold[1] or abs(add_angs(pos[2], -self.goal_pos[2])) >= 360-goal_threshold[1]
            self.at_goal_pos = lambda pos: math.dist(pos[:2], self.goal_pos[:2]) < goal_threshold_xy and ang(pos)
            
            # Check if angles meet threshold
            self.angle_thresh = lambda theta, theta_i: abs(theta-theta_i) > self.angle_threshold and abs(theta-theta_i) <= 360-self.angle_threshold
         
        # Append frame
        self.frames.append(self.grid)
        
        # Differentiate between cost path planning algorithm (e.g. Djikstra, A*) and other FIFO (e.g. BFS)
        orig_parent_loc_string = Search.pos2str(self.start_pos)
        if cost_algorithm:
            self.node_info = {orig_parent_loc_string : {self.start_pos[2] : {"cost_from_first" : 0}}}          # Dictionary of node info
            
            # Define current set priority queue for node inspection
            self.current_set = [(0, self.start_pos)]
            heapq.heapify(self.current_set)
            
            # Define queue manipulators
            self.q_set = lambda cost, pos: heapq.heappush(self.current_set, (cost, pos))
            self.q_get = lambda: heapq.heappop(self.current_set)
        else:
            self.node_info = {orig_parent_loc_string : {self.start_pos[2] : {}}}                                # Dictionary of node info
            
            # Define current set queue for node inspection
            self.current_set = queue.Queue()
            
            # Define queue manipulators
            self.q_set = lambda pos: self.current_set.put_nowait(pos)
            self.q_get = lambda: self.current_set.get_nowait()
            self.q_set(self.start_pos)
                
        # Other 
        self.add_frame_frequency = round(add_frame_frequency/8)        
        self.uint8_0 = np.uint8(0)                                                       # Define a 0 as an 8-bit unsigned int. This will save processing time    
        self.radius *= scale
        self.clearence *= scale 
        self.action_set = action_set["action_set"]
          
    # Generate robot move action set
    @staticmethod      
    def gen_robot_action_set(node_threshold_xy=0.5, node_threshold_theta=30, goal_threshold_xy=1.5, goal_threshold_theta=30, max_add_turn_cost=0):
        print("\n=====================================================================================\n")
        
        dist_def = 5
        dist_bounds = (1, 10)
        
        theta_def = 30
        theta_bounds = (1, 179)
        
        n_branches_def = 5
        n_branch_bounds = (1, 10)
        
        # Query user for positions
        def get_user_input(pos_string, default, bounds):
            prompt = None
            if pos_string == "dist":
                prompt = f"Enter step size distance ({bounds[0]} <= step size <= {bounds[1]}) or leave blank to apply the default value: "
            elif pos_string == "theta":
                prompt = f"Enter angle theta ({bounds[0]} < theta < {bounds[1]}) or leave blank to apply the default value: "
            else:
                prompt = f"Enter number of possible branches (should be odd) or leave blank to apply the default value: "
                        
            # Query until user has input legal values     
            while True:
                start_str = input(prompt)
                start_str_nw = start_str.replace(" ", "")
                
                # If empty string, set values to default
                if start_str_nw == '':
                    print("Selecting default value: ", default)
                    return default              
                                    
                # Check for incorrect input
                try:
                    val = int(start_str_nw)
                except:
                    print("Please type a single integer.\n")
                    continue    
                
                # Check if position out of bounds
                if val < bounds[0] or val > bounds[1]:
                    print("Numbers out of bounds. Please select new value.\n")
                    continue

                # User chose correct inputs
                return val
        
        
        
        # Define start/goal positions
        dist = get_user_input("dist", dist_def, dist_bounds)
        print("")
        theta = get_user_input("theta", theta_def, theta_bounds)        
        print("")
        n_branches = get_user_input("branches", n_branches_def, n_branch_bounds)            
        print("\n=====================================================================================")
        
        # Init dict
        action_set = {
            "discrete" : False,
            "node_threshold" : (node_threshold_xy, node_threshold_theta),
            "goal_threshold" : (goal_threshold_xy, goal_threshold_theta),
            "action_set" : {}
        }
        # Apply action set
        even = False
        if n_branches%2 == 0:
            even = True
              
        dist /= node_threshold_xy
        max_turn = None
        turn_defined = False
        for i in range(n_branches):
            t = None
            if even:
                t = (n_branches//2 - i)*theta - theta//2
            else:
                t = (n_branches//2 - i)*theta 
            if not turn_defined:
                turn_defined = True
                max_turn = t
            action_set["action_set"][i] = {'move' : (dist, t), "cost" : dist+abs(max_add_turn_cost*t/max_turn)}
        return action_set
            
    # Convert position tuple into string for dictionary storage
    @staticmethod
    def pos2str(pos):
        return str(pos[0]) + " " + str(pos[1])  
    
    # Check if obstacles in range
    def no_obstacles(self, curr_pos):
        # Current position on obstacle
        if self.grid[curr_pos[1], curr_pos[0]][-1] != self.uint8_0:
            return False
        
        # No range for obstacles
        if self.radius + self.clearence == 0:
            return True
            
        # Total radius 
        r = self.radius + self.clearence
        
        # Bounds
        from_x = int(max(0, curr_pos[0]-r))
        to_x = int(min(self.width-1, curr_pos[0]+r))
        from_y = int(max(0, curr_pos[1]-r))
        to_y = int(min(self.height-1, curr_pos[1]+r))
        
        # Find nearby red pixels
        wall_indeces = np.where(self.grid[from_y:to_y, from_x:to_x, -1] != self.uint8_0)
        for x, y in zip(wall_indeces[0]-r, wall_indeces[1]-r):
            if np.linalg.norm([x,y]) < r:
                return False
        return True
    
    # Check if we've visited a given node/angle
    def not_visited(self, node_info, theta):
        if node_info == None:
            return True
        
        if not self.discrete:
            if np.all([self.angle_thresh(theta, theta_i) for theta_i in node_info.keys()]):
                return True
        
        return False
    
    # Draw pixels or arrows
    def draw_on_grid(self, parent, pos, color):
        if self.discrete:
            self.grid[pos[1], pos[0]] = color
        else:
            cv2.arrowedLine(self.grid, parent[:2], pos[:2], color, 2)
                   
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
                    # self.show_grid(self.grid, wait=1)
                    count = 0
            
            print("\n=====================================================================================")
            if solution_found:
                # Go up the dictionary tree to find all parent nodes and optimized path
                next_set = self.node_info[Search.pos2str(self.winning_loc)][self.winning_loc[2]]
                next_set_ind = next_set['parent']
                path = [next_set_ind]
                try:
                    while True:
                        next_set_ind = next_set['parent']
                                                
                        next_set = self.node_info[Search.pos2str(next_set_ind)][next_set_ind[2]]                        
                        path.append(next_set_ind)
                                                   
                        # Change grid color and add frames to video
                        cv2.circle(self.grid, (next_set_ind[0], next_set_ind[1]), 2, (0,255,0), -1)
                        cv2.circle(self.grid, (next_set_ind[0], next_set_ind[1]), 2, (0,255,0), -1)
                        self.grid[(next_set_ind[1], next_set_ind[0])] = (0,255,0)
                        self.frames.append(self.grid.copy())
                        
                        if next_set_ind[0] == self.start_pos[0] and next_set_ind[1] == self.start_pos[1]:
                            break
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
                print(f"\nThe {self.__class__.__name__} algorithm cannot detect a path. Please select new start and goal points or remove obstacles from the grid.")
               
            print("\nSaving Video...")
            # Save and write video feed
            self.save(self.__class__.__name__ + '_' + self.video_name + "_animation", True)
            print(f"The video file can be found in your current directory under {self.__class__.__name__}_{self.video_name}_animation.avi")
            print("\n=====================================================================================\n")

            # Display grid
            Search.show_grid(self.grid, self.scale_percent, wait=10000)   
               
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            self.save(self.video_name + "_animation_INTERRUPT", True)
