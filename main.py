#<=============================== Imports ===================================>#
import os
import math
import search
import trial_maps as maps

#<=============================== Main ===================================>#
if __name__ == '__main__':
    os.system('cls')

    # Action set for discrete moves
    action_set = {
        "action_set" : {
            "up"         : {"move":  (0,-1), "cost": 1},
            "up_right"   : {"move":  (1,-1), "cost": math.sqrt(2)},
            "right"      : {"move":   (1,0), "cost": 1},
            "down_right" : {"move":   (1,1), "cost": math.sqrt(2)},
            "down"       : {"move":   (0,1), "cost": 1},
            "down_left"  : {"move":  (-1,1), "cost": math.sqrt(2)},
            "left"       : {"move":  (-1,0), "cost": 1},
            "up_left"    : {"move": (-1,-1), "cost": math.sqrt(2)},
        },
        "discrete"       : True,
    }
    
    #  Action set for turning robots
    action_set = search.Search.gen_robot_action_set(
        node_threshold_xy = 0.5,                                    # Euclidean distance threshold for considering two nodes having the same pos
        node_threshold_theta = 30,                                  # Threshold for considering two nodes having the same angle
        goal_threshold_xy = 1.5,                                    # Euclidean distance threshold for node being close enough to goal pos
        goal_threshold_theta = 30,                                  # Angle threshold for node being close enough to goal angle
        max_add_turn_cost = 0                                       # Added cost for maximum turn. Intermediate turns will have proportional costs between the propulsion radius, d <= turn_cost <= d + max_add_turn_cost  
    )
    
    # A* Searches
    trial_map = search.AStarSearch(maps.TrialMap, action_set, scale_percent=300,add_frame_frequency=50, framerate=100)
    trial_map.find_path()
    
    # cv2.destroyAllWindows()

    # submission_map = search.AStarSearch(maps.SubmissionMap, action_set, scale_percent=200, add_frame_frequency=300, framerate=100)
    # submission_map.find_path()
    
    # cv2.destroyAllWindows()
    
    # test_no_path_map = search.AStarSearch(maps.TestNoPathMap, action_set, scale_percent=300, add_frame_frequency=50, framerate=100)
    # test_no_path_map.find_path()
        
    # cv2.destroyAllWindows()

    # Dijikstra Searches
    # trial_map = search.DijkstraSearch(maps.TrialMap, action_set, scale_percent=300,add_frame_frequency=50, framerate=100)
    # trial_map.find_path()
    
    # cv2.destroyAllWindows()

    # submission_map = search.DijkstraSearch(maps.SubmissionMap, action_set, scale_percent=200, add_frame_frequency=300, framerate=100)
    # submission_map.find_path()
    
    # cv2.destroyAllWindows()
    
    # test_no_path_map = search.DijkstraSearch(maps.TestNoPathMap, action_set, scale_percent=300, add_frame_frequency=50, framerate=100)
    # test_no_path_map.find_path()
        
    # cv2.destroyAllWindows()
    
    # Breadth-First Searches
    # trial_map = search.BFSSearch(maps.TrialMap, action_set, scale_percent=300,add_frame_frequency=50, framerate=100)
    # trial_map.find_path()
    
    # cv2.destroyAllWindows()

    # submission_map = search.BFSSearch(maps.SubmissionMap, action_set, scale_percent=200, add_frame_frequency=300, framerate=100)
    # submission_map.find_path()
    
    # cv2.destroyAllWindows()
    
    # test_no_path_map = search.BFSSearch(maps.TestNoPathMap, action_set, scale_percent=300, add_frame_frequency=50, framerate=100)
    # test_no_path_map.find_path()