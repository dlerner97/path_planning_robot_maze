#!/usr/bin/python3
import os
import math
import rospy
import search
import trial_maps as maps
from nav_msgs import Odometry
from geometry_msgs import Twist

class ROSIntegration:
    def __init__(self, search_type, map, action_set, scale_percent=100, add_frame_frequency=100, framerate=100) -> None:
        self.search_algorithm = search_type(map, action_set, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate)

    def odom_callback(self, msg):
        self.position = msg.pose.position

    def find_path(self):
        self.search_algorithm.find_path()
        self.path = self.search_algorithm.path
        self.action_path = self.search_algorithm.action_path
        print(len(self.path))
        print(len(self.action_path))

    def publish_path(self, dist_threshold=0.01, cmd_topic='/cmd_vel', pose_topic="/odom", pub_rate=10):
        subscriber = rospy.Subscriber(pose_topic, Odometry, callback=self.odom_callback, queue_size=10)
        pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        rate = rospy.Rate(pub_rate)

        for way_point, action in zip(self.path, self.action_path):
            while not rospy.is_shutdown:
                
                dist = math.dist(way_point[:2], self.position[:2])
                if dist < dist_threshold:
                    break

                msg = self.get_twist_msg(action)
                pub.publish(msg)
                rate.sleep()
                
if __name__ == '__main__':
    os.system('cls')
    rospy.init_node("path planner", anonymous=True)

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

    ros_path_plan = ROSIntegration(search.AStarSearch, maps.TrialMap, action_set, scale_percent=300,add_frame_frequency=50, framerate=100)
    ros_path_plan.find_path()
    # ros_path_plan.publish_path()

