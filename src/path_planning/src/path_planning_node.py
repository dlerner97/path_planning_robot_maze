#!/usr/bin/python3

# =================== Imports =================== # 
import os
import cv2
import math
import time
import rospy
import search
import numpy as np
import trial_maps as maps
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from action_generator import RobotActionSetGenerator
from tf.transformations import euler_from_quaternion

# =================== ROSIntegration Class Definition =================== # 
class ROSIntegration:

    """
    ROS Integration Class

    args: search_type, map, action_set, scale_percent=100, add_frame_frequency=100, framerate=100

    Integrates search algorithms with ROS/Gazebo simulation.
    """

    def __init__(self, search_type, map, action_set, scale_percent=100, add_frame_frequency=100, framerate=100) -> None:
        
        # Initialize search
        self.search_algorithm = search_type(map, action_set, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate)
        self.got_odom = False

        # Initialize action set
        self.discrete = False
        if action_set["move_type"] == "discrete":
            self.discrete = True
        elif action_set["move_type"] == "rt":
            pass
        elif action_set["move_type"] == "diff drive":
            pass
        else:
            print("Please ensure that the 'move_type' key in action_set is a valid driving type.")
            raise ValueError

        int_ = lambda val: int(round(val))
        origin_offset = None

        # Find origin offset
        try:
            origin_offset = rospy.get_param("origin_offset")
        except:
            origin_offset = (5,5)

        self.origin_offset = (int_(origin_offset[0]), int_(origin_offset[1]))

    # Odometry subscriber callback function
    def odom_callback(self, msg):
        # Position
        self.got_odom = True
        pos_msg = msg.pose.pose.position
        self.position = [pos_msg.x+self.origin_offset[0], pos_msg.y+self.origin_offset[1], pos_msg.z]
        
        # Orientation
        orient_msg = msg.pose.pose.orientation
        orient_vec = [orient_msg.x, orient_msg.y, orient_msg.z, orient_msg.w]
        self.orientation = np.degrees(list(euler_from_quaternion(orient_vec)))

    # Run search algorithm
    def find_path(self):
        self.search_algorithm.find_path()
        self.path = self.search_algorithm.path
        self.action_path = self.search_algorithm.action_path

    # RT Controller -> Turn towards next point then traverse
    def move_rt(self, action, way_point, dist_threshold=0.05, angle_threshold=0.1, speed=0.1, turn_speed=0.2):               
        print("\nMove rt...")
        print(10*'-')

        # Find turn angle
        theta_d = math.degrees(math.atan2(way_point[1]-self.position[1], way_point[0]-self.position[0]))
        turn_angle = theta_d - self.orientation[-1]
        
        turn_dir = -1
        if turn_angle < -180 or (turn_angle > 0 and turn_angle < 180):
            turn_dir = 1

        # Turn
        msg = Twist()
        count = 10
        print("Turning...")
        while not rospy.is_shutdown():
            if count >= 10:
                print(10*"=")
                print("curr orientation:", self.orientation[-1])
                print("way_pt:", theta_d)
                print("dist:", theta_d - self.orientation[-1])
                count = 0               
            if turn_dir == 1 and self.orientation[-1] >= theta_d - angle_threshold:
                print("Stop turning!")
                break
            elif turn_dir == -1 and self.orientation[-1] <= theta_d + angle_threshold:
                print("Stop turning!")
                break

            msg.angular.z = turn_dir*turn_speed
            self.pub.publish(msg)
            self.angle_rate.sleep()        

        # Move forward
        msg = Twist()
        count = 10
        print("Moving Forward...")
        while not rospy.is_shutdown():
            dist = math.dist(way_point[:2], self.position[:2])
            if count >= 10:
                print(10*"=")
                print("curr pos:", self.position)
                print("way_pt:", way_point)
                print("dist:", dist)
                count = 0
            
            if dist < dist_threshold:
                print("At way point")
                break

            msg.linear.x = speed
            self.pub.publish(msg)
            count += 1
            self.rate.sleep()

    # Closed loop controller
    def move_diff_drive(self, way_point, dist_threshold=0.05):

        # Turn and traverse at the same time
        msg = Twist()
        # count = 0

        # Gains
        k_lin = 1
        k_ang = .05

        # Go to next waypoint
        while not rospy.is_shutdown():
            dist = (math.dist(way_point[:2], self.position[:2]), math.degrees(math.atan2(way_point[1]-self.position[1], way_point[0]-self.position[0])))

            # if count >= 10:
            #     print(10*"=")
            #     print("curr pos:", self.position)
            #     print("way_pt:", way_point)
            #     print("dist:", dist)
            #     count = 0
            
            if dist[0] < dist_threshold:
                print("At way point")
                break

            msg.linear.x = k_lin*dist[0]
            msg.angular.z = k_ang*(dist[1]-self.orientation[-1])

            self.pub.publish(msg)
            # count += 1
            self.rate.sleep()

        msg.linear.x = 0
        msg.angular.z = 0
        self.pub.publish(msg)

    # Publishes path in closed loop controller to turtlebot
    def publish_path(self, cmd_topic='/cmd_vel', pose_topic="/odom", lin_pub_rate=50, ang_pub_rate=50):

        print("\nRelaying path to turtlebot...")

        # Declare odom subscriber
        self.subscriber = rospy.Subscriber(pose_topic, Odometry, callback=self.odom_callback, queue_size=10)
        
        # Declare cmd_vel publisher
        self.pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)

        # Declare ROS rates
        self.rate = rospy.Rate(lin_pub_rate)
        self.angle_rate = rospy.Rate(ang_pub_rate)

        # Wait for first odometry message
        while not self.got_odom:
            self.rate.sleep()

        # Traverse all way points
        for way_point, action in zip(self.path[1:], self.action_path[1:]):
            way_point = np.array(way_point)
            # self.move_rt(action, way_point)
            self.move_diff_drive(action, way_point)
            if(rospy.is_shutdown()):
                print("Keyboard Interrupt")
                break
        
        if not rospy.is_shutdown():
            print(10*'-', "At goal!", 10*'-')
            self.pub.publish(Twist())
        cv2.destroyAllWindows()

# =================== Node Main =================== # 
if __name__ == '__main__':
    # Init ROS Node
    rospy.init_node("path_planner", anonymous=True)

    # Wait for parameter definitions
    time.sleep(5)
    # os.system('clear')

    # Action set for discrete moves
    # action_set = RobotActionSetGenerator.gen_robot_discrete_action_set(
    #     diagnol = True,
    #     move_amt = 5
    # )
    
    #  Action set for turning robots
    # action_set = RobotActionSetGenerator.gen_robot_rt_action_set(
    #     node_threshold_xy = 0.5,                                    # Euclidean distance threshold for considering two nodes having the same pos
    #     node_threshold_theta = 30,                                  # Threshold for considering two nodes having the same angle
    #     goal_threshold_xy = 1.5,                                    # Euclidean distance threshold for node being close enough to goal pos
    #     goal_threshold_theta = 30,                                  # Angle threshold for node being close enough to goal angle
    #     max_add_turn_cost = 0                                       # Added cost for maximum turn. Intermediate turns will have proportional costs between the propulsion radius, d <= turn_cost <= d + max_add_turn_cost  
    # )
    
    # Declare action set
    action_set = RobotActionSetGenerator.gen_robot_diff_drive_action_set(
        node_threshold_xy = 0.5,                                    # Euclidean distance threshold for considering two nodes having the same pos
        node_threshold_theta = 30,                                  # Threshold for considering two nodes having the same angle
        goal_threshold_xy = 1.5,                                    # Euclidean distance threshold for node being close enough to goal pos
        goal_threshold_theta = 30                                   # Angle threshold for node being close enough to goal angle
    )

    # Run all
    ros_path_plan = ROSIntegration(search.AStarSearch, maps.GazeboMap, action_set, scale_percent=300,add_frame_frequency=50, framerate=100)
    ros_path_plan.find_path()
    ros_path_plan.publish_path()

