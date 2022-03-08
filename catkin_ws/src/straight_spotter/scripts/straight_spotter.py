#!/usr/bin/env python3

from dataclasses import dataclass
import enum
import math

import threading
from tracemalloc import stop
from turtle import distance

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

message = """
Available commands:
/set_movement_speed NUMBER      - sets movement speed
                                in meters per second

/set_rotation_speed NUMBER      - sets rotation spped
                                in angle per second

/set_distance_precision NUMBER - sets distance precision
                                in meters, a.k.a. condition
                                of movement stop

/set_yaw_precision NUMBER      - sets rotation precision
                                in angle, a.k.a condition
                                of rotation stop

/go_to x:NUMBER y:NUMBER        - move robot to position
                                (x, y) on meters scale

/stop_movement                  - stop movement to desired
                                position

/continue_movement              - continue movement to desired
                                position

/get_status                     - prints current robot state

/help                           - prints this message
"""


class Robot:
    class State(enum.Enum):
        move = 0
        idle = 1

    def __init__(self, movement_speed=0.5, rotation_speed=.7, distance_precision=0.1, yaw_precision=.5):
        self.position = Point()
        self.yaw = 0.0

        self.distance_precision = distance_precision
        self.yaw_precision = yaw_precision

        self.movement_speed = movement_speed
        self.rotation_speed = rotation_speed
        
        self.desired_position = Point()
        self.state = Robot.State.idle

    def handle_odometry(self, msg: Odometry):
        self.position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = transformations.euler_from_quaternion(quaternion)        
        self.yaw = euler[2]


    def fix_yaw(self):
        desired_yaw = math.atan2(
            self.desired_position.y - self.position.y,
            self.desired_position.x - self.position.x
        )

        delta_yaw = desired_yaw - self.yaw

        if math.fabs(delta_yaw) > self.yaw_precision:
            twist_msg = Twist()
            twist_msg.angular.z = -self.rotation_speed if delta_yaw > 0 else self.rotation_speed
            return twist_msg
        else:
            return None

    def go_ahead(self):
        distance_to_desired = math.sqrt((self.desired_position.y - self.position.y)**2 + (self.desired_position.x - self.position.x)**2)
        if distance_to_desired > self.distance_precision:
            twist_msg = Twist()
            twist_msg.linear.x = self.movement_speed
            return twist_msg
        else:
            self.state = Robot.State.idle
            return None

    def request_twist(self):
        if self.state == Robot.State.move:
            return self.fix_yaw() or self.go_ahead()
        stop_twist = Twist()
        stop_twist.angular.z = 0
        stop_twist.linear.x = 0
        return stop_twist

class RobotController(threading.Thread):
    def __init__(self):
        super(RobotController, self).__init__()
        self.robot = Robot()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.robot.handle_odometry)
        self.done = False
        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print(f"Waiting for subscriver to connect to {self.publisher.name}")
            rospy.sleep(0.5)
            i += 1
            i %= 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def stop(self):
        self.done = True
        self.join()

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and not self.done:
            twist_msg = self.robot.request_twist()
            if twist_msg:
                self.publisher.publish(twist_msg)
            rate.sleep()

    def set_movement_speed(self, NUMBER):
        self.robot.movement_speed = NUMBER

    def set_rotation_speed(self, NUMBER):
        self.robot.rotation_speed = NUMBER

    def set_distance_precision(self, NUMBER):
        self.robot.distance_precision = NUMBER

    def set_yaw_precision(self, NUMBER):
        self.robot.yaw_precision = NUMBER

    def go_to(self, x_NUMBER, y_NUMBER):
        self.robot.desired_position.x = x_NUMBER
        self.robot.desired_position.y = y_NUMBER
        self.robot.state = Robot.State.move

    def stop_movement(self):
        self.robot.state = Robot.State.idle

    def continue_movement(self):
        self.robot.state = Robot.State.move

    def get_status(self):
        print(f"state: {self.robot.state.name}")
        print("")
        print(f"precision: distance={self.robot.distance_precision}, yaw={self.robot.yaw_precision}")
        print(f"speed:     movement={self.robot.movement_speed}, rotation={self.robot.rotation_speed}")
        print("")
        print(f"desired position: ({self.robot.desired_position.x}, {self.robot.desired_position.y})")
        print(f"current position: ({self.robot.position.x}, {self.robot.position.y})")
        print(f"current yaw:      {self.robot.yaw}")

    def help(self):
        print(message)


def main():
    rospy.init_node('straight_spotter')

    robot_controller = RobotController()
    robot_controller.help()
    robot_controller.wait_for_subscribers()
    
    while not rospy.is_shutdown():
        line = input()
        if line[0] != "/":
            print("! unrecognized input !")
            print("! print /help")
        else:
            c, *args = line[1:].split(" ")
            try:
                getattr(robot_controller, c)(*map(float, args))
            except (AttributeError, TypeError, ValueError) as e:
                print(e)


if __name__ == "__main__":
    main()