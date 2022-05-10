#!/usr/bin/env python3

from time import sleep
import rospy
import sys
from os.path import abspath, dirname, exists
from os import makedirs
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

import rosbag_recorder


def main():
    rospy.init_node('goal_provider')
    x = rospy.get_param("~x", 0)
    y = rospy.get_param("~y", 0)
    theta = rospy.get_param("~theta", 0)
    record = rospy.get_param("~record", "")

    ws = "/".join(abspath(__file__).split("/")[:-4])
    rosbag_dir = f"{ws}/rosbag/{record}"
    if not exists(rosbag_dir):
        makedirs(rosbag_dir)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    while(not client.wait_for_server(rospy.Duration(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()


    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, theta))


    rosbag_recorder.RosbagRecord("rosbag record /odom /move_base/GlobalPlanner/plan /robot/cmd_vel", rosbag_dir)
    
    rospy.loginfo("Sending goal")
    client.send_goal(goal)
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("done!")
    else:
        rospy.loginfo("failure!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)