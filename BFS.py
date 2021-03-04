import cv2
import time
import queue
import operator
import numpy as np

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
# This class handles the breadth first search (BFS) algorithm 
class BFSSearch(VideoBuffer):
    # Takes in video_name, default start/goal positions, output feed scale factor, number of nodes between subsequent frames, video framerate
    def __init__(self, vid_name, start_pos_def, goal_pos_def, scale_percent=100, add_frame_frequency=100, framerate = 100):
        # Init VideoBuffer Parent
        super().__init__(framerate=framerate, scale_percent=scale_percent)
        
        # Query user for positions
        def get_user_input(pos_string, default):
            
            # Query until user has input legal values     
            while True:
                start_str = input(f"Enter {pos_string} position as x,y (0 <= x <= {self.width-1} ',' and 0 <= y <= {self.height-1}) or just enter for default: ")
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
        
        print("\nSelect image window and press any key to begin.\nHit 'ctrl+C' at any time to quit execution.")
        
        # Add start/goal positions to grid
        temp_grid = self.grid.copy()
        cv2.circle(temp_grid, self.start_pos, 3, (0,255,0), -1)
        cv2.circle(temp_grid, self.goal_pos, 3, (0,255,0), -1)
        self.grid = temp_grid
        
        # Display grid
        self.scale_percent = scale_percent
        BFSSearch.show_grid(self.grid, scale_percent)
        cv2.destroyAllWindows()
        self.frames.append(self.grid)
        
        # Define variables
        orig_parent_loc_string = str(self.start_pos[0]) + " " + str(self.start_pos[1])
        self.parent_info = {orig_parent_loc_string : {}}        # Dictionary of node info
        
        self.current_set = queue.Queue()                                                    # Queue of states to inspect 
        self.current_set.put_nowait(self.start_pos)
         
        self.add_frame_frequency = round(add_frame_frequency/8)
        self.video_name = vid_name
        
        self.uint8_0 = np.uint8(0)                                                          # Define a 0 as an 8-bit unsigned int. This will save processing time    
          
    # Convert position tuple into string for dictionary storage
    @staticmethod
    def pos2str(pos):
        return str(pos[0]) + " " + str(pos[1]) 

    # Gets next branch in the given direction
    def get_next_branch(self, parent, from_dir):
        
        # Get the new position using tuple addition
        pos = tuple(map(operator.add, parent, (from_dir)))

        # Check if new position is on the grid
        on_grid =             pos[1] < self.height
        on_grid = on_grid and pos[1] >= 0
        on_grid = on_grid and pos[0] < self.width
        on_grid = on_grid and pos[0] >= 0 
        
        # Check if move is legal and generate that new state
        if (on_grid):
            loc_string = BFSSearch.pos2str(pos) 
            parent_loc_string = BFSSearch.pos2str(parent)
            
            # If position corresponds to the winning state, return True
            if pos[0] == self.goal_pos[0] and pos[1] == self.goal_pos[1]:
                self.parent_info[loc_string] = {"parent": parent_loc_string}
                return True
            
            # Check if the node has already been visited or if it is on an obstacle
            color = self.grid[pos[1], pos[0]]
            if self.parent_info.get(loc_string) == None and color[-1] == self.uint8_0:
                
                # Change pixel color to blue unless it overlaps with the start and end depictions
                if color[1] == self.uint8_0:
                    self.grid[pos[1], pos[0]] = (255,0,0)
                
                # Update queue and parent info
                self.parent_info[loc_string] = {"parent": parent_loc_string}
                self.current_set.put_nowait(pos)
        return False
    
    # Check all subsequent directions for a given position
    def check_subsequent_nodes(self, parent):

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
        start_time = time.time()                                # Time path planning
        solution_found = False
        count = 0
        try:
            while (not solution_found):
                
                # If queue is empty, stop
                if self.current_set.empty():
                    break
                
                # Pull state from queue and check subsequent nodes
                parent = self.current_set.get_nowait()
                solution_found = self.check_subsequent_nodes(parent)
                
                # Only add frame to video at specified intervals
                count += 1
                if count == self.add_frame_frequency:
                    self.frames.append(self.grid.copy())
                    count = 0
            
            if solution_found:
                # Go up the dictionary tree to find all parent nodes and optimized path
                next_set = self.parent_info[BFSSearch.pos2str(self.goal_pos)]
                str_list = BFSSearch.pos2str(self.goal_pos).split(" ")
                num_list = [int(str_list[0]), int(str_list[1])]
                path = [num_list]
                try:
                    while True:
                        next_set_ind = next_set['parent']
                        next_set = self.parent_info[next_set_ind]
                        
                        # Parse strings and add them to path
                        str_list = next_set_ind.split(" ")
                        num_list = [int(str_list[0]), int(str_list[1])]
                        path.append(num_list)
                        
                        # Change grid color and add frames to video
                        self.grid[num_list[1], num_list[0]] = (0,255,0)
                        self.frames.append(self.grid.copy())
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
                print(f"The BFS algorithm took {time_elapsed_hrs} hrs, {time_elapsed_mins} mins, and {time_elapsed_secs} s to implement.\n")
              
            # If no solution was found
            else:
                print("\nThe BFS algorithm cannot detect a path. Please select new start and goal points or remove obstacles from the grid.")
               
            # Save and write video feed
            self.save(self.video_name + "_animation", True)
            print(f"The video file can be found in your current directory under {self.video_name}_animation.avi")

            # Display grid
            BFSSearch.show_grid(self.grid, self.scale_percent)   
               
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
