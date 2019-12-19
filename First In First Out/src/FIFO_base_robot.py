#!/usr/bin/env python

from math import atan2, pi, pow, sqrt
import random
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Int32, String
import ActionType as types
from FIFO_base_info import Data
from tf.transformations import euler_from_quaternion

from rocon_std_msgs.msg._StringArray import StringArray

class Run_Node(Data):
    def __init__(self, name, init_position, destination):
        super(Run_Node, self).__init__(name, init_position, destination)

        # ros init
        rospy.init_node('centralize_tb3_' + self.name, anonymous=True)
        self.r = rospy.Rate(10)


        self.signal = ''
        self.ready = types.NOT_READY
        self.all_direction = ['']

        self.entered_once = True
        self.catch_up = None
        self.prior_speed = 0.0

        # speed publisher
        self.cmd_vel = rospy.Publisher('tb3_' + self.name +'/cmd_vel', Twist, queue_size=5)
        odom = rospy.Subscriber('tb3_' + self.name +'/odom', Odometry, self.callback_odom)

        self.sys_ready = rospy.Subscriber('ready', String, self.callback_system)

        self.same_direction = []

        self.safe_distance = 0.35

        # assign speed
        self.tb3_move_cmd = Twist()
        self.max_speed = 0.3
        self.speed = round(random.uniform(0.1, 0.5), 2)

        if self.speed > self.max_speed:
            self.speed = self.max_speed

        self.end_destination = 0.0

        # self.speed = 0.15

        print('Robot {}: {}'.format(self.name, self.speed))

        # location publisher
        self.location = rospy.Publisher('tb3_' + self.name + '/direction', String, queue_size=5)
        self.direction = self.calc_direction(self.initial_position, self.destination)

        while self.ready == types.NOT_READY:
            self.location.publish(self.direction)

        # unsubscribe to system signal
        self.sys_ready.unregister()

        # get all the direction from other vehicles
        self.total_direction = rospy.wait_for_message('total_direction', StringArray)
        self.all_direction = self.total_direction.strings

        # publish our own command
        self.self_command = rospy.Publisher('tb3_' + self.name + '/self_command', String, queue_size=5)

        # print(self.all_direction)

        for i in range(len(self.all_direction)):
            if self.all_direction[i] == self.direction and str(i) != self.name:
                self.same_direction.append(i)

        self.same_direction_location_x = [0.0] * len(self.same_direction)
        self.same_direction_location_y = [0.0] * len(self.same_direction)

        # id of the vehicles in front
        self.in_front = None
        self.prior_command = ''

        self.other_location = []

        for i in self.same_direction:
            self.other_location.append(rospy.Subscriber('/tb3_' + str(i) + '/odom', Odometry, self.callback_other_odom, (self.same_direction.index(i))))
            
        # wait for the signal
        while 0.0 in self.same_direction_location_x and 0.0 in self.same_direction_location_y:
            pass

        print("Robot {}: friend {} -> {}, {}".format(self.name, self.same_direction, self.same_direction_location_x, self.same_direction_location_y))


        # method to get the front vehicle id
        if self.verticle_direction():
            self.closest_x = self.find_closest_distance(self.same_direction_location_x, self.initial_position[0])
            if self.closest_x != 0 and self.get_distance(min(self.same_direction_location_y), self.initial_position[1]) <= 0.2:
                self.in_front = self.same_direction[self.same_direction_location_x.index(self.closest_x)]

        else:
            self.closest_y = self.find_closest_distance(self.same_direction_location_y, self.initial_position[1])
            if self.closest_y != 0 and self.get_distance(min(self.same_direction_location_x), self.initial_position[0]) <= 0.2:
                self.in_front = self.same_direction[self.same_direction_location_y.index(self.closest_y)]

        # subscribe to the speed of the vehicle in front
        if self.in_front is not None:
            print('Robot {} is infront of: {}'.format(self.in_front, self.name))
            prior_vehicle_odom = rospy.Subscriber('/tb3_' + str(self.in_front) + '_command', String, self.callback_prior_command)
            prior_vehicle_speed = rospy.Subscriber('tb3_' + str(self.in_front) + '/cmd_vel', Twist, self.callback_prior_speed)
            self.catch_up = False

        # unsubscribe to unnecessary topic
        for i in self.same_direction:
            if i != self.in_front:
                self.other_location[self.same_direction.index(i)].unregister()


        rospy.sleep(1)

        signal_data = rospy.Subscriber('tb3_'+ self.name +'_command', String, self.callback_signal)

        self.in_front_moved = True

        while not rospy.is_shutdown():
            # guide the robot toward the direction
            self.rotate()
            self.position = self.current_position[0] if self.verticle_direction() else self.current_position[1]

            # adjust the speed when it's get too close to the font vehicle
            if self.in_front is not None and self.prior_command == types.MOVING:
                # get the front vehicle position
                if self.verticle_direction():
                    self.prior_position = self.same_direction_location_x[self.same_direction.index(self.in_front)]
                else:
                    self.prior_position = self.same_direction_location_y[self.same_direction.index(self.in_front)]

                if self.get_distance(self.position, self.prior_position) <= (self.safe_distance + 0.1):
                    # make sure the prior speed doesn't change
                    if self.speed > self.prior_speed and self.prior_speed != 0.0:
                        self.speed = self.prior_speed

            # handle multiple signal coming from the front vehicle
            if self.prior_command == types.STOP:
                self.in_front_moved = True
                if self.get_distance(self.prior_position, self.position) <= self.safe_distance:
                     # stop immendiately
                    self.publish_signal(types.STOP)


            elif self.prior_command == types.ENTER_INTERSECTION and self.entered_once:
                self.entered_once = False
                # start moving when the other vehicles enter the area
                self.publish_signal(types.MOVING)

            elif self.prior_command == types.MOVING and self.in_front_moved:
                self.in_front_moved = False
                self.publish_signal(types.MOVING)

            # * choose_status is working
            self.choose_status(self.signal)

            # * Shutdown the node when it reaches the destination
            if self.reached_destination():
                rospy.signal_shutdown('Passed Intersection!')
                rospy.on_shutdown(self.myhook)


    def choose_status(self, x):
        return {
            types.MOVING: self.Start,
            types.ENTER_INTERSECTION: self.Enter_intersection,
            types.PASS_INTERSECTION: self.Pass_intersection,
            types.STOP: self.Stop,
        }.get(x, self.Stop)()


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

    def get_distance(self, a, b):
        return abs(abs(a) - abs(b))

    def find_closest_distance(self, l, t):
        closest_values = [x for x in l if abs(x) < abs(t)]
        if closest_values:
            return max(closest_values) if t > 0 else min(closest_values)
        return 0

    def publish_signal(self, x):
        self.signal = x
        self.self_command.publish(x)
            
    def myhook(self):
        print("shutdown time for {}!".format(self.name))


    def Start(self):
        # assign random speed (could add acceleration speed later)
        self.publish_speed_signal(self.speed)


    def Stop(self):
        # set speed
        self.publish_speed_signal(0.0)

    # Enter and Pass intersection currently is the same as Start
    def Enter_intersection(self):
        # assign random speed (could add acceleration speed later)
        self.publish_speed_signal(self.speed)


    def Pass_intersection(self):
        # assign random speed (could add acceleration speed later)
        self.publish_speed_signal(self.speed)


    def rotate(self):
        path_angle = atan2(self.destination[1] - self.current_position[1], self.destination[0] - self.current_position[0])

        self.tb3_move_cmd.angular.z = path_angle - self.rotation
        if self.tb3_move_cmd.angular.z > pi:
            self.tb3_move_cmd.angular.z = self.tb3_move_cmd.angular.z - 2 * pi
        if self.tb3_move_cmd.angular.z < -pi:
            self.tb3_move_cmd.angular.z = self.tb3_move_cmd.angular.z + 2 * pi
        if self.tb3_move_cmd.angular.z > 0:
            self.tb3_move_cmd.angular.z = min(
                self.tb3_move_cmd.angular.z, 1.5)
        else:
            self.tb3_move_cmd.angular.z = max(
                self.tb3_move_cmd.angular.z, -1.5)

        self.cmd_vel.publish(self.tb3_move_cmd)


    def publish_speed_signal(self, x):
        self.tb3_move_cmd.linear.x = x
        self.cmd_vel.publish(self.tb3_move_cmd)
        self.r.sleep()


    def reached_destination(self):
        if self.signal == types.PASS_INTERSECTION:
            self.end_destination = self.destination[0] if self.verticle_direction() else self.destination[1]

            if abs(self.position) > (abs(self.end_destination) - 0.5):
                return True
        return False

    def verticle_direction(self):
        if self.direction == types.DIR_LEFT or self.direction == types.DIR_RIGHT:
            return True
        return False

########################################### callback function
    def callback_odom(self, msg):
        self.current_position[0], self.current_position[1] = msg.pose.pose.position.x, msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.rotation) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def callback_signal(self, msg):
        self.signal = msg.data

    def callback_system(self, msg):
        self.ready = msg.data
        
    def callback_other_odom(self, msg, args):
        self.same_direction_location_x[args] = round(msg.pose.pose.position.x, 2)
        self.same_direction_location_y[args] = round(msg.pose.pose.position.y, 2)

    def callback_prior_command(self, msg):
        self.prior_command = msg.data

    def callback_prior_speed(self, msg):
        self.prior_speed = msg.linear.x