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


class Robot:
    class State(enum.Enum):
        idle = 0
        fix_heading = 1
        move = 2

    def __init__(self, movement_speed=0.5, rotation_speed=.7, distance_precision=0.3, yaw_precision=.1):
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
            return -self.rotation_speed if delta_yaw > 0 else self.rotation_speed
        else:
            return 0

    def go_ahead(self):
        distance_to_desired = math.sqrt((self.desired_position.y - self.position.y)**2 + (self.desired_position.x - self.position.x)**2)
        if distance_to_desired > self.distance_precision:
            return self.movement_speed
        else:
            return 0

    def request_twist(self):
        twist = Twist()
        twist.angular.z = 0
        twist.linear.x = 0
        if self.state == Robot.State.move or self.state == Robot.State.fix_heading:
            twist.angular.z  = self.fix_yaw()
            if twist.angular.z == 0:
                self.state = Robot.State.move

        if self.state == Robot.State.move:
            twist.linear.x = self.go_ahead()
            if twist.linear.x == 0:
                self.state = Robot.State.idle

        return twist



class CommandHandler:
    commands = {}
    def runCommand(self, name, *args):
        try:
            return CommandHandler.commands[name]["member"](self, *args)
        except KeyError as e:
            raise AttributeError(f"Unknown command: /{name}")

def command(description):
        def __command_wrapper(func):
            name = func.__name__
            CommandHandler.commands[name] = {
                "description": description,
                "member": func
            }
            return func
        return __command_wrapper

class RobotController(threading.Thread, CommandHandler):
    commands = {}

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

        print(CommandHandler.commands)
    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and not self.done:
            twist_msg = self.robot.request_twist()
            if twist_msg:
                self.publisher.publish(twist_msg)
            rate.sleep()

    @command("sets movement speed in meters per second")
    def set_movement_speed(self, NUMBER):
        self.robot.movement_speed = NUMBER

    @command("sets rotation spped in angle per second")
    def set_rotation_speed(self, NUMBER):
        self.robot.rotation_speed = NUMBER

    @command("sets distance precision in meters, a.k.a. condition of movement stop")
    def set_distance_precision(self, NUMBER):
        self.robot.distance_precision = NUMBER

    @command("sets rotation precision in angle, a.k.a condition of rotation stop")
    def set_yaw_precision(self, NUMBER):
        self.robot.yaw_precision = NUMBER

    @command("move robot to position (x, y) on meters scale")
    def go_to(self, x_NUMBER, y_NUMBER):
        self.robot.desired_position.x = x_NUMBER
        self.robot.desired_position.y = y_NUMBER
        self.robot.state = Robot.State.fix_heading

    @command("stop movement to desired position")
    def stop_movement(self):
        self.robot.state = Robot.State.idle

    @command("continue movement to desired position")
    def continue_movement(self):
        self.robot.state = Robot.State.move

    @command("prints current robot state")
    def get_status(self):
        print(f"state: {self.robot.state.name}")
        print("")
        print(f"precision: distance={self.robot.distance_precision}, yaw={self.robot.yaw_precision}")
        print(f"speed:     movement={self.robot.movement_speed}, rotation={self.robot.rotation_speed}")
        print("")
        print(f"desired position: ({self.robot.desired_position.x}, {self.robot.desired_position.y})")
        print(f"current position: ({self.robot.position.x}, {self.robot.position.y})")
        print(f"current yaw:      {self.robot.yaw}")

    @command("prints this message")
    def help(self):
        width = 0
        for command_name in CommandHandler.commands:
            if len(command_name) > width:
                width = len(command_name)

        for command_name, command in CommandHandler.commands.items():
            print(f"/{command_name.ljust(width)} - {command['description']}")


def main():
    rospy.init_node('straight_spotter')

    robot_controller = RobotController()
    robot_controller.help()
    robot_controller.wait_for_subscribers()
    
    while not rospy.is_shutdown():
        line = input()
        if len(line) == 0:
            continue
        if line[0] != "/":
            print("! unrecognized input !")
            print("! print /help")
        else:
            c, *args = line[1:].split(" ")
            try:
                robot_controller.runCommand(c, *map(float, args))
            except (AttributeError, TypeError, ValueError) as e:
                print(e)


if __name__ == "__main__":
    main()