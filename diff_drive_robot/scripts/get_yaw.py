#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np

current_pose = [0.0, 0.0, 0.0]

def cbSlamPose(pose_msg):
    current_pose[0] = pose_msg.pose.position.x
    current_pose[1] = pose_msg.pose.position.y
    orientation = pose_msg.pose.orientation
    euler_pose = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    current_pose[2] = euler_pose[2]
    print(current_pose[2])

if __name__ == "__main__":
    rospy.init_node("yaw_checker")
    rospy.loginfo("Node initialized")

    sub_slam_pose = rospy.Subscriber("/slam_out_pose", PoseStamped, cbSlamPose)


    rospy.sleep(2)


    while True:
        rospy.spin()