#<=============================== Imports ===================================>#
import cv2
import math
import numpy as np

#<=============================== TrialMap Class Definition ===================================>#
class TrialMap:  
    
    """
    Trial Map
    
    This class builds the trial map and can be passed into the search algorithms as parameters
    """
         
    # Build grid with red obstacles
    @staticmethod
    def gen_grid():
        width = 200
        height = 100
        
        grid = np.zeros((height,width, 3), np.uint8)                # Initialize shape
        grid[40-1: 60, 90-1:110] = (0,0,255)                        # Rectangle
        center_circ = (width-1-40, 50-1)
        cv2.circle(grid, center_circ, 15, (0,0,255), -1)            # Circle
        return "trial_map", grid[::-1,:], height, width        
        
#<=============================== SubmissionMap Class Definition ===================================>#
class SubmissionMap:
    
    """
    Submission Map
    
    This class builds the submission map and can be passed into the search algorithms as parameters
    """
    
    # Build grid with red obstacles
    @staticmethod
    def gen_grid():
        width = 400
        height = 300
        grid = np.zeros((height, width, 3), np.uint8)                # Initialize shape
        
        # Circle
        center_circ = (90-1, 70-1)
        cv2.circle(grid, center_circ, 35, (0,0,255), -1)
        
        # Utility lambda functions
        deg2rad = lambda deg: deg*math.pi/180
        int_ = lambda val: int(round(val))
        
        # Rectangle
        lower_left_corner = [48-1, 108-1]
        angle = deg2rad(35)
        upper_left_corner = [int_(lower_left_corner[0] + 20*math.cos(angle+deg2rad(90)))-1, int_(lower_left_corner[1] + 20*math.sin(angle+deg2rad(90)))-1]
        lower_right_corner = [int_(lower_left_corner[0] + 150*math.cos(angle))-1, int_(lower_left_corner[1] + 150*math.sin(angle))-1]
        upper_right_corner = [int_(upper_left_corner[0] + 150*math.cos(angle))-1, int_(upper_left_corner[1] + 150*math.sin(angle))-1]        
        pts = np.array([upper_left_corner, lower_left_corner, lower_right_corner, upper_right_corner], np.int32).reshape((-1,1,2))
        cv2.fillPoly(grid, [pts], (0,0,255))
        
        # Ellipse
        center_ell = (246-1, 145-1)
        cv2.ellipse(grid, center_ell, (120//2, 60//2), 0, 0, 360, (0,0,255), -1)
        
        # Top polygon
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
        
        # Right polygon
        # angle_low = deg2rad(45)
        # bot = [width-1-72, 63-1]
        # right_x = bot[0] + int_(75*math.cos(angle_low))
        # right_bot_y = bot[1] + int_(75*math.sin(angle_low))
        # mid_right = [right_x, right_bot_y]
        # top_right = [right_x, right_bot_y+55]
        # left = [bot[0]+int_(60*math.cos(angle_low+deg2rad(90))), bot[1]+int_(60*math.sin(angle_low+deg2rad(90)))]
        # top_mid = [left[0]+int_(60*math.cos(angle_low)), left[1]+int_(60*math.sin(angle_low))]
        # bot_mid = [354-1, 138-1]
        # pts = np.array([bot, mid_right, top_right, bot_mid, top_mid, left], np.int32).reshape((-1,1,2))
        # cv2.fillPoly(grid, [pts], (0,0,255))
            
        return "submission_map", grid[::-1,:], height, width  
      
#<=============================== TestNoPathMap Class Definition ===================================>#
class TestNoPathMap:
    
    """
    No Possible Path Map
    
    This class builds the "Test No Path" map and can be passed into the search algorithms as parameters
    
    This class builds a map with no path between the left and right-hand sides.
    It displays the the algorithms ability to understand that no path is possible 
    """
    
    # Build grid with red obstacles
    @staticmethod
    def gen_grid():
        width = 200
        height = 100
        
        grid = np.zeros((height,width, 3), np.uint8)                # Initialize shape
        grid[:, 90:100] = (0,0,255)
        
        return "no_path_map", grid[::-1,:], height, width