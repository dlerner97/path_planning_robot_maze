#!/usr/bin/python3
import os
import math
import rospy
import search
import trial_maps as maps
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from action_generator import RobotActionSetGenerator
from tf.transformations import euler_from_quaternion

class ROSIntegration:
    def __init__(self, search_type, map, action_set, scale_percent=100, add_frame_frequency=100, framerate=100) -> None:
        self.search_algorithm = search_type(map, action_set, scale_percent=scale_percent, add_frame_frequency=add_frame_frequency, framerate=framerate)
        self.got_odom = False

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

    def odom_callback(self, msg):
        self.got_odom = True
        pos_msg = msg.pose.pose.position
        self.position = [pos_msg.x, pos_msg.y, pos_msg.z]
        
        orient_msg = msg.pose.pose.orientation
        orient_vec = [orient_msg.x, orient_msg.y, orient_msg.z, orient_msg.w]
        self.orientation = euler_from_quaternion(orient_vec)

    def find_path(self):
        self.search_algorithm.find_path()
        self.path = self.search_algorithm.path
        self.action_path = self.search_algorithm.action_path

    def move_rt(self, action, way_point, dist_threshold=0.01, speed=.1, angle_threshold=0.5):               
        theta_d = action[1]
        if self.discrete:
            theta_d = math.degrees(math.atan2(action[1], action[0]))

        turn_angle = theta_d - self.orientation[-1]
        turn_dir = -1
        if turn_angle < -180 or (turn_angle > 0 and turn_angle < 180):
            turn_dir = 1

        msg = Twist()
        print("Turning...")
        while not rospy.is_shutdown():
            
            turn_angle = theta_d - self.orientation[-1]
            if turn_dir == 1 and self.orientation[-1] >= theta_d - angle_threshold:
                break
            elif turn_dir == -1 and self.orientation[-1] <= theta_d + angle_threshold:
                break

            msg.angular.z = turn_dir*speed
            self.pub.publish(msg)
            self.rate.sleep()        

        msg = Twist()
        print("Moving Forward...")
        while not rospy.is_shutdown():
            print(way_point)
            print(self.position)
            dist = math.dist(way_point[:2], self.position[:2])
            if dist < dist_threshold:
                break

            msg.linear.x = speed
            self.pub.publish(msg)
            self.rate.sleep()

    def publish_path(self, dist_threshold=0.01, cmd_topic='/cmd_vel', pose_topic="/odom", pub_rate=10):
        self.subscriber = rospy.Subscriber(pose_topic, Odometry, callback=self.odom_callback, queue_size=10)
        self.pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        self.rate = rospy.Rate(pub_rate)

        while not self.got_odom:
            self.rate.sleep()

        for way_point, action in zip(self.path[1:], self.action_path[1:]):
            print("Next way point")
            self.move_rt(action, way_point)

                
if __name__ == '__main__':
    # os.system('cls')
    rospy.init_node("path_planner", anonymous=True)

    # Action set for discrete moves
    action_set = RobotActionSetGenerator.gen_robot_discrete_action_set(
        diagnol = True,
        move_amt = 1
    )
    
    #  Action set for turning robots
    # action_set = RobotActionSetGenerator.gen_robot_rt_action_set(
    #     node_threshold_xy = 0.5,                                    # Euclidean distance threshold for considering two nodes having the same pos
    #     node_threshold_theta = 30,                                  # Threshold for considering two nodes having the same angle
    #     goal_threshold_xy = 1.5,                                    # Euclidean distance threshold for node being close enough to goal pos
    #     goal_threshold_theta = 30,                                  # Angle threshold for node being close enough to goal angle
    #     max_add_turn_cost = 0                                       # Added cost for maximum turn. Intermediate turns will have proportional costs between the propulsion radius, d <= turn_cost <= d + max_add_turn_cost  
    # )
    
    ros_path_plan = ROSIntegration(search.AStarSearch, maps.TrialMap, action_set, scale_percent=300,add_frame_frequency=50, framerate=100)
    ros_path_plan.find_path()
    ros_path_plan.publish_path()

