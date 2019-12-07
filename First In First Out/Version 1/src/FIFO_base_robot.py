#!/usr/bin/env python

from math import atan2, pi, pow, radians, sqrt
import time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Int32, String, Float32
from tf.transformations import euler_from_quaternion
import random
import ActionType as types
from FIFO_base_info import Data


class Run_Node(Data):
    def __init__(self, name, init_position, destination):
        super(Run_Node, self).__init__(name, init_position, destination)

        # ros init
        rospy.init_node('centralize_tb3_' + self.name, anonymous=True)
        self.r = rospy.Rate(10)
        
        # speed publisher
        self.cmd_vel = rospy.Publisher('tb3_' + self.name +'/cmd_vel', Twist, queue_size=5)
        self.odom = rospy.Subscriber('tb3_' + self.name +'/odom', Odometry, self.callback_odom)

        # assign speed
        self.tb3_move_cmd = Twist()
        self.speed = round(random.uniform(0.1, 0.3), 2)

        print('Robot {}: {}'.format(self.name, self.speed))

        # location publisher
        location = rospy.Publisher('tb3_' + self.name + '/position', String, queue_size=5)
        self.direction = self.calc_direction(self.initial_position, self.destination)

        print('Robot {} Direction: {}'.format(self.name, self.direction))

        # rotate to desired location
        # self.Rotate()

        rospy.sleep(1)

        while not rospy.is_shutdown():
            location.publish(self.direction)
            signal_data = rospy.Subscriber('tb3_'+ self.name +'_command', String, self.callback_signal)
            self.Start() if self.signal == types.MOVING else self.Stop()
            self.r.sleep()

    # calculate the direction
    def calc_direction(self, start, end):
        if start[0] < end[0] and self.check_abs(start[0], start[1]):
            return types.DIR_RIGHT
        elif start[0] > end[0] and self.check_abs(start[0], start[1]):
            return types.DIR_LEFT
        elif start[1] < end[1] and self.check_abs(start[1], start[0]):
            return types.DIR_UP
        else:
            return types.DIR_DOWN


    def check_abs(self, x, y):
        if abs(x) > abs(y):
            return True
        return False

    def Start(self):
        # assign random speed (could add acceleration speed later)
        self.tb3_move_cmd.linear.x = self.speed
        self.cmd_vel.publish(self.tb3_move_cmd)

    def Stop(self):
        # set speed
        self.tb3_move_cmd.linear.x = 0.0
        self.cmd_vel.publish(self.tb3_move_cmd)


    # doesn't work really well currently
    def Rotate(self):
        while True:
            self.x = self.destination[0] - self.current_position[0]
            self.y = self.destination[1] - self.current_position[1]

            self.angle_to_dest = atan2(self.y, self.x)

            if abs(self.angle_to_dest - self.rotation) > 0.005:
                self.tb3_move_cmd.linear.x = 0.0
                self.tb3_move_cmd.angular.z = 0.3
            else:
                self.tb3_move_cmd.angular.z = 0.0 
                self.cmd_vel.publish(self.tb3_move_cmd)
                break

            self.cmd_vel.publish(self.tb3_move_cmd)


    # callback function
    def callback_odom(self, msg):
        self.current_position[0], self.current_position[1] = msg.pose.pose.position.x, msg.pose.pose.position.y

        # rot_q = msg.pose.pose.orientation
        # (roll, pitch, self.rotation) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def callback_signal(self, msg):
        self.signal = msg.data