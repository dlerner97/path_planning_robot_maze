import os
import cv2
import search
import trial_maps as maps

if __name__ == '__main__':
    os.system('cls')

    # trial_map = search.AStarSearch(maps.TrialMap, 10, 5, scale_percent=300,add_frame_frequency=50, framerate=100)
    # trial_map.find_path()
    
    # cv2.destroyAllWindows()

    submission_map = search.AStarSearch(maps.SubmissionMap, 10, 5, scale_percent=200, add_frame_frequency=300, framerate=100)
    submission_map.find_path()
    
    # cv2.destroyAllWindows()
    
    # test_no_path_map = search.AStarSearch(maps.TestNoPathMap, 10, 5, scale_percent=300, add_frame_frequency=50, framerate=100)
    # test_no_path_map.find_path()
        
    # cv2.destroyAllWindows()

    # trial_map = search.DijkstraSearch(maps.TrialMap, 10, 5, scale_percent=300,add_frame_frequency=50, framerate=100)
    # trial_map.find_path()
    
    # cv2.destroyAllWindows()

    # submission_map = search.DijkstraSearch(maps.SubmissionMap, 10, 5, scale_percent=200, add_frame_frequency=300, framerate=100)
    # submission_map.find_path()
    
    # cv2.destroyAllWindows()
    
    # test_no_path_map = search.DijkstraSearch(maps.TestNoPathMap, 10, 5, scale_percent=300, add_frame_frequency=50, framerate=100)
    # test_no_path_map.find_path()
        
    # cv2.destroyAllWindows()
    
    # trial_map = search.BFSSearch(maps.TrialMap, 10, 5, scale_percent=300,add_frame_frequency=50, framerate=100)
    # trial_map.find_path()
    
    # cv2.destroyAllWindows()

    # submission_map = search.BFSSearch(maps.SubmissionMap, 10, 5, scale_percent=200, add_frame_frequency=300, framerate=100)
    # submission_map.find_path()
    
    # cv2.destroyAllWindows()
    
    # test_no_path_map = search.BFSSearch(maps.TestNoPathMap, 10, 5, scale_percent=300, add_frame_frequency=50, framerate=100)
    # test_no_path_map.find_path()