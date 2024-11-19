#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np

current_pose = [0.0, 0.0, 0.0]
goal = [0.98, 0.634]


stop = Twist()
stop.linear.x = 0.0
stop.linear.y, stop.linear.z = 0.0, 0.0
stop.angular.z = 0.0
stop.angular.x, stop.angular.y = 0.0, 0.0

turn_left = Twist()
turn_left.linear.x = 0.0
turn_left.linear.y, turn_left.linear.z = 0.0, 0.0
turn_left.angular.z = 0.1
turn_left.angular.x, turn_left.angular.y = 0.0, 0.0

turn_right = Twist()
turn_right.linear.x = 0.0
turn_right.linear.y, turn_right.linear.z = 0.0, 0.0
turn_right.angular.z = -0.1
turn_right.angular.x, turn_right.angular.y = 0.0, 0.0


go_forward = Twist()
go_forward.linear.x = 0.04
go_forward.linear.y, go_forward.linear.z = 0.0, 0.0
go_forward.angular.z = 0.0
go_forward.angular.x, go_forward.angular.y = 0.0, 0.0

go_backward = Twist()
go_backward.linear.x = -0.01
go_backward.linear.y, go_backward.linear.z = 0.0, 0.0
go_backward.angular.z = 0.0
go_backward.angular.x, go_backward.angular.y = 0.0, 0.0

def cbSlamPose(pose_msg):
    current_pose[0] = pose_msg.pose.position.x
    current_pose[1] = pose_msg.pose.position.y
    orientation = pose_msg.pose.orientation
    euler_pose = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    current_pose[2] = euler_pose[2]
    # rospy.loginfo("Received pose: ")
    # print(current_pose)

def stop_robot(time):

    begin = rospy.Time.now()
    delay = rospy.Duration(time)
    end = begin + delay

    while rospy.Time.now() < end:
        pub_cmd.publish(stop)

    rospy.loginfo("Current Yaw: %f", current_pose[2])


def turn_robot_left(time):

    begin = rospy.Time.now()
    delay = rospy.Duration(time)
    end = begin + delay

    while rospy.Time.now() < end:
        pub_cmd.publish(turn_left)

    rospy.loginfo("Current Yaw: %f", current_pose[2])


def turn_robot_right(time):

    begin = rospy.Time.now()
    delay = rospy.Duration(time)
    end = begin + delay

    while rospy.Time.now() < end:
        pub_cmd.publish(turn_right)

    rospy.loginfo("Current Yaw: %f", current_pose[2])

def turn_robot(target_rad):
    if (target_rad - current_pose[2]) >0:
        while abs(current_pose[2] - target_rad) > 0.05:
            turn_robot_left(0.08)
            stop_robot(0.08)
    else:
        while abs(current_pose[2] - target_rad) > 0.05:
            turn_robot_right(0.08)
            stop_robot(0.08)

    rospy.loginfo("Current Yaw: %f", current_pose[2])


if __name__ == "__main__":
    rospy.init_node("cmd_publisher")
    rospy.loginfo("Node initialized")

    pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    sub_slam_pose = rospy.Subscriber("/slam_out_pose", PoseStamped, cbSlamPose)



    rospy.sleep(2)


    rospy.loginfo("Turn left")
    turn_robot(0.83)


    stop_robot(1.0)


    rospy.loginfo("Turn right")

    turn_robot(0.57)

    stop_robot(0.8)


    rospy.loginfo("Move forward")
    start_trans = current_pose[0:2]
    offset = np.sqrt((start_trans[0])**2 + (start_trans[1])**2)

    dist = np.sqrt((start_trans[0]-current_pose[0])**2 + (start_trans[1]-current_pose[1])**2)

    # need about 0.2m to stop
    while dist + offset < 1.1:
        pub_cmd.publish(go_forward)
        dist = np.sqrt((start_trans[0]-current_pose[0])**2 + (start_trans[1]-current_pose[1])**2)
        rospy.loginfo("Moved distance: %f", dist)

    stop_robot(1.8)

    np.sqrt((goal[0]-current_pose[0])**2 + (goal[1]-current_pose[1])**2)
    rospy.loginfo("Distance to goal: %f", np.sqrt((goal[0]-current_pose[0])**2 + (goal[1]-current_pose[1])**2))

    