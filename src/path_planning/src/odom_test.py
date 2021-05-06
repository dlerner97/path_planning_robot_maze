#!/usr/bin/python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

if __name__ == '__main__':
    # Init ROS Node
    rospy.init_node("odom_test", anonymous=True)
    orientation = None

    def odom_callback(msg):
        global orientation
        orient_msg = msg.pose.pose.orientation
        orient_vec = [orient_msg.x, orient_msg.y, orient_msg.z, orient_msg.w]
        orientation = np.degrees(euler_from_quaternion(orient_vec)[-1])
        # print(self.orientation[-1])

    subscriber = rospy.Subscriber("/odom", Odometry, callback=odom_callback, queue_size=10)
    while not rospy.is_shutdown():
        rospy.loginfo(f"------------------------")
        rospy.loginfo(f"yaw: {orientation}")
        rospy.sleep(1)
        