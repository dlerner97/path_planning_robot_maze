#<=============================== Imports ===================================>#
import cv2
import time
import math
import queue
import heapq
import numpy as np
from roslib.packages import get_pkg_dir

try:
    from rospy import get_param
except:
    print("ROS is not installed on this computer. Code will only be affected IFF the ROS node is run. If the script is run from main.py, the code should still work.")

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
        try:
            grid.shape
        except:
            grid = grid.get().copy()

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
            f = cv2.UMat(f)
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
        self.int_ = lambda val: int(round(val))
        print("\n=====================================================================================\n")
        
        # Define map
        map_dict = map_.gen_grid()
        self.video_name = map_dict["name"]
        self.grid = map_dict["grid"]
        self.height = map_dict["height"]
        self.width = map_dict["width"]
        self.map_scaling = map_dict["map_scaling"]
        start_pos_def = map_dict["map_def_start"]
        goal_pos_def = map_dict["map_def_goal"]
        robot_scaling = map_dict["robot_scale"]
        
        self.move_type = action_set["move_type"]
        if action_set["move_type"] == "discrete":
            self.discrete = True
        else:
            self.discrete = False
        
        # Query user for positions
        def get_pos_user_input(pos_string, default):
            # Query until user has input legal values     
            while True:
                start_str = f"Enter {pos_string} position as x,y(,theta) (0 <= x <= {self.width/self.map_scaling - 1} ',' 0 <= y <= {self.height/self.map_scaling-1}" 
                if not self.discrete:
                    start_str += ", 0 <= theta <= 359"
                
                start_str = input(start_str + ") or leave blank to apply the default values: ")
                start_str_nw = start_str.replace(" ", "")
                
                # If empty string, set values to default
                if start_str_nw == '':
                    print("Selecting default values: ", (default[0], default[1], 0))
                    print("")
                    return (self.int_(self.map_scaling*default[0]), self.int_(self.height-1-self.map_scaling*default[1]), 0)              
                    
                start_str_list = start_str_nw.split(",")
                
                # Check for incorrect input
                try:
                    x_pos = int(self.map_scaling*start_str_list[0])
                    y_pos = (self.height-1-int(self.map_scaling*start_str_list[1]))
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
                
                print("")
                # User chose correct inputs
                if self.discrete:
                        return (x_pos, y_pos, 0)
                return (x_pos, y_pos, theta)
            
        def get_user_input(pos_string, default=0):                       
            # Query until user has input legal values     
            while True:
                start_str = input(f"Enter robot base {pos_string} (0 <= {pos_string}) or leave blank to apply the default values: ")
                start_str_nw = start_str.replace(" ", "")
                
                # If empty string, set values to default
                if start_str_nw == '':
                    print("Selecting default value: ", default)
                    print("")
                    return default              
                                    
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

                print("")
                # User chose correct inputs
                return val
        
        # Define start/goal positions
        dt = 0
        try: 

            start_pos = get_param("start_pos")
            goal_pos = get_param("goal_pos")
            
            set_up_pos = lambda pos: (self.int_(self.map_scaling*pos[0]), self.int_(self.height-1-self.map_scaling*pos[1]), self.int_(pos[2]))
            self.start_pos = set_up_pos(start_pos)
            self.goal_pos = set_up_pos(goal_pos)
            self.radius = self.int_(self.map_scaling*get_param("robot_radius"))
            if action_set["move_type"] == "diff drive":
                self.wheel_radius = robot_scaling*get_param("wheel_radius")
                self.robot_width = robot_scaling*get_param("robot_width")
                dt = get_param("diff_drive_time")
            
        except:
            print("rosparams not available. Please select start and goal positions from the terminal.\n")
            self.start_pos = get_pos_user_input("start", start_pos_def)                     # Start position
            self.goal_pos = get_pos_user_input("goal", goal_pos_def)                        # Goal position
            self.radius = get_user_input("radius")                                          # Robot radius
            if action_set["move_type"] == "diff drive":
                self.wheel_radius = robot_scaling*get_user_input("wheel radius", 0.038)
                self.robot_width = robot_scaling*get_user_input("robot width", 0.354)
                dt = get_user_input("diff_drive_time", 0.1)

        self.clearence = get_user_input("clearence")                                    # Robot clearence
        print("=====================================================================================")
        
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
        self.scale = 1
        
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
            cos = lambda angle: math.cos(math.radians(angle))
            sin = lambda angle: math.sin(math.radians(angle))
            add_angs = lambda angle1, angle2: (angle1+angle2+2*360)%360

            # Scale grid to account for xy distance threshold
            node_threshold = action_set["node_threshold"]
            self.scale = 1/node_threshold[0]

            if action_set["move_type"] == "rt":
                # Define new position from previous and change
                self.get_next_pos = lambda parent, change: (int(round(parent[0]+change[0]*cos(add_angs(change[1],parent[2])))), 
                                                            int(round(parent[1]-change[0]*sin(add_angs(change[1],parent[2])))), add_angs(parent[2],change[1]))
            
            elif action_set["move_type"] == "diff drive":
                self.wheel_radius *= self.scale*self.map_scaling
                self.robot_width *= self.scale*self.map_scaling

                def get_next_pos(parent, change):
                    t = 0
                    x_n = parent[0]
                    y_n = parent[1]
                    theta_n = parent[2]
                    self.intermediate_x = []
                    self.intermediate_y = []
                    while t < 1:
                        t += dt
                        x_s = x_n
                        y_s = y_n

                        # print("-----")
                        # print("change", change)
                        # print("theta_n", theta_n)
                        # print("x_n add:", 0.5*self.wheel_radius*(change[0]+change[1])*cos(theta_n)*dt) 
                        # print("y_n add:", 0.5*self.wheel_radius*(change[0]+change[1])*sin(theta_n)*dt)
                        # print("theta_n add:", math.degrees((self.wheel_radius/self.robot_width)*(change[1]-change[0])*dt))
                         
                        x_n += 0.5*self.wheel_radius*(change[0]+change[1])*cos(theta_n)*dt
                        y_n += 0.5*self.wheel_radius*(change[0]+change[1])*sin(theta_n)*dt
                        theta_n = int(round(add_angs(theta_n, math.degrees((self.wheel_radius/self.robot_width)*(change[1]-change[0])*dt))))
                        # print('theta_n', theta_n)
                        # print("-----") 
                        self.intermediate_x.append([x_s, x_n])
                        self.intermediate_y.append([y_s, y_n])

                    return (self.int_(x_n), self.int_(y_n), theta_n)
                
                for key, values in action_set["action_set"].items():
                    t = 0
                    cost = 0
                    theta_n = 0
                    change = values["move"]
                    while t < 1:
                        t += dt                     
                        x_n = 0.5*self.wheel_radius*(change[0]+change[1])*cos(theta_n)*dt
                        y_n = 0.5*self.wheel_radius*(change[0]+change[1])*sin(theta_n)*dt
                        theta_n = add_angs(theta_n, math.degrees((self.wheel_radius/self.robot_width)*(change[1]-change[0])*dt))
                        cost += math.sqrt(x_n**2 + y_n**2)
                    action_set["action_set"][key]["cost"] = cost

                self.get_next_pos = lambda parent, change: get_next_pos(parent, change)

            self.grid = self.resize_image(self.grid, 100*self.scale)
            self.width *= self.scale
            self.height *= self.scale
            self.goal_pos = (int(self.scale*self.goal_pos[0]), int(self.scale*self.goal_pos[1]), self.goal_pos[2])
            self.start_pos = (int(self.scale*self.start_pos[0]), int(self.scale*self.start_pos[1]), self.start_pos[2]) 
            self.scale_percent *= node_threshold[0]
            self.angle_threshold = node_threshold[1]
            
            # Check if at goal position
            goal_threshold = action_set['goal_threshold']
            goal_threshold_xy = self.scale*goal_threshold[0]
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
        self.radius *= self.scale
        self.clearence *= self.scale 
        self.action_set = action_set["action_set"]
                      
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
        elif self.move_type == "rt":
            cv2.arrowedLine(self.grid, parent[:2], pos[:2], color, 2)
        elif self.move_type == "diff drive":
            for x, y in zip(self.intermediate_x, self.intermediate_y):
                cv2.line(self.grid, (self.int_(x[0]), self.int_(y[0])), (self.int_(x[1]), self.int_(y[1])), color, 1)

                   
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
                action_path = [next_set['action']]
                if not self.discrete:
                    self.grid = cv2.UMat(self.grid)
                
                try:
                    while True:
                        next_set_ind = next_set['parent']
                        
                        print(10*"=")
                        print("ind", next_set_ind)
                                                
                        next_pos = self.node_info[Search.pos2str(next_set_ind)]
                        next_set = None
                        try:
                            next_set = next_pos[next_set_ind[2]]
                        except KeyError:
                            print(next_pos)
                            keys = list(next_pos.keys())
                            print("keys", keys)
                            closest_theta = keys[np.argmin([not self.angle_thresh(next_set_ind[2], theta_i) for theta_i in keys])]
                            next_set = next_pos[closest_theta]

                        print("next set theta", next_set)                        
                        path.append(next_set_ind)
                        action_path.append(next_set["action"])
                        # self.show_grid(self.grid)
                        print(10*"=")
         
                        # Change grid color and add frames to video
                        if self.discrete:
                            self.grid[(next_set_ind[1], next_set_ind[0])] = (0,255,0)
                        else:
                            cv2.circle(self.grid, (next_set_ind[0], next_set_ind[1]), 2, (0,255,0), -1)
                            cv2.circle(self.grid, (next_set_ind[0], next_set_ind[1]), 2, (0,255,0), -1)
                            self.frames.append(self.grid)

                        if next_set_ind[0] == self.start_pos[0] and next_set_ind[1] == self.start_pos[1]:
                            break
                except KeyError:
                    print("Done?")

                if not self.discrete:
                    self.grid = self.grid.get()

                # Print full move set
                path.reverse()
                map_scale = lambda val: round(val/self.map_scaling, 1)
                path = map(lambda path_i: (map_scale(path_i[0]/self.scale), map_scale((self.height-1-path_i[1])/self.scale), path_i[2]), path)
                self.path = np.asarray(list(path))

                action_path.append((0,0))
                action_path.reverse()
                self.action_path = action_path

                # self.grid = self.grid.get()
                # self.grid[self.path[:, 1], self.path[:, 0]] = (0,255,0)
                
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
            full_path = get_pkg_dir('path_planning') + "/avi_vids/"
            # Save and write video feed
            self.save(full_path + self.__class__.__name__ + '_' + self.video_name + "_animation", True)
            print(f"The video file can be found in the path_planning package as avi_vids/{self.__class__.__name__}_{self.video_name}_animation.avi")
            print("\n=====================================================================================\n")

            # Display grid
            Search.show_grid(self.grid, self.scale_percent, wait=10000)   
               
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            self.save(self.video_name + "_animation_INTERRUPT", True)
