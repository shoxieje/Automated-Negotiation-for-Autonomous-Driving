#!/usr/bin/env python

import random
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Int32, String
import ActionType as types
from FIFO_base_info import Data

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

        # speed publisher
        self.cmd_vel = rospy.Publisher('tb3_' + self.name +'/cmd_vel', Twist, queue_size=5)
        odom = rospy.Subscriber('tb3_' + self.name +'/odom', Odometry, self.callback_odom)

        sys_ready = rospy.Subscriber('ready', String, self.callback_system)

        self.same_direction = []

        self.safe_distance = 0.5

        # assign speed
        self.tb3_move_cmd = Twist()
        # self.speed = round(random.uniform(0.1, 0.3), 2)
        self.speed = 0.15

        print('Robot {}: {}'.format(self.name, self.speed))


        # location publisher
        self.location = rospy.Publisher('tb3_' + self.name + '/direction', String, queue_size=5)
        self.direction = self.calc_direction(self.initial_position, self.destination)

        while self.ready == types.NOT_READY:
            self.location.publish(self.direction)

        self.total_direction = rospy.wait_for_message('total_direction', StringArray)
        self.all_direction = self.total_direction.strings


        # self.command = types.MOVING
        self.self_command = rospy.Publisher('tb3_' + self.name + '/self_command', String, queue_size=5)

        print(self.all_direction)

        for i in range(len(self.all_direction)):
            if self.all_direction[i] == self.direction and str(i) != self.name:
                self.same_direction.append(i)

        self.same_direction_location_x = [0.0] * len(self.same_direction)
        self.same_direction_location_y = [0.0] * len(self.same_direction)

        # id of the vehicles in front
        self.in_front = None
        self.prior_command = ''

        for i in self.same_direction:
            other_location = rospy.Subscriber('/tb3_' + str(i) + '/odom', Odometry, self.callback_other_odom, (self.same_direction.index(i)))
            
        # wait for the signal
        while 0.0 in self.same_direction_location_x and 0.0 in self.same_direction_location_y:
            pass

        print("Robot {}: friend {} -> {}, {}".format(self.name, self.same_direction, self.same_direction_location_x, self.same_direction_location_y))


        # method to get the front vehicle id
        for x in self.same_direction_location_x:
            if self.direction == types.DIR_LEFT or self.direction == types.DIR_RIGHT:
                if abs(min(self.same_direction_location_x)) < abs(self.initial_position[0]) and self.get_distance(min(self.same_direction_location_y), self.initial_position[1]) <= 0.2:
                    self.in_front = self.same_direction[self.same_direction_location_x.index(x)]

        for y in self.same_direction_location_y:
            if self.direction == types.DIR_DOWN or self.direction == types.DIR_UP:
                if abs(min(self.same_direction_location_y)) < abs(self.initial_position[1]) and self.get_distance(min(self.same_direction_location_x), self.initial_position[0]) <= 0.2:
                    self.in_front = self.same_direction[self.same_direction_location_y.index(y)]
    
        if self.in_front is not None:
            prior_vehicle = rospy.Subscriber('/tb3_' + str(self.in_front) + '_command', String, self.callback_prior_command)
        else:
            del self.in_front

        # rotate to desired location
        # self.Rotate()

        rospy.sleep(1)

        signal_data = rospy.Subscriber('tb3_'+ self.name +'_command', String, self.callback_signal)

        while not rospy.is_shutdown():
            if self.prior_command == types.STOP:
                if self.direction == types.DIR_RIGHT or self.direction == types.DIR_LEFT:
                    # get the distance between the current vehicle and the one in front of it
                    if self.same_direction_location_x[self.same_direction.index(self.in_front)] - self.current_position[0] <= self.safe_distance:
                        # stop immediately
                        self.signal = types.STOP
                        self.self_command.publish(types.STOP)

            elif self.prior_command == types.ENTER_INTERSECTION:
                # start moving when the other vehicles enter the area
                self.signal = types.MOVING
                self.self_command.publish(types.MOVING)
            
            # * choose_status is working
            self.choose_status(self.signal)


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


    def Start(self):
        # assign random speed (could add acceleration speed later)
        self.tb3_move_cmd.linear.x = self.speed
        self.cmd_vel.publish(self.tb3_move_cmd)

    def Stop(self):
        # set speed
        self.tb3_move_cmd.linear.x = 0.0
        self.cmd_vel.publish(self.tb3_move_cmd)


    # Enter and Pass intersection currently is the same as Start
    def Enter_intersection(self):
        # assign random speed (could add acceleration speed later)
        self.tb3_move_cmd.linear.x = self.speed
        self.cmd_vel.publish(self.tb3_move_cmd)

    def Pass_intersection(self):
        # assign random speed (could add acceleration speed later)
        self.tb3_move_cmd.linear.x = self.speed
        self.cmd_vel.publish(self.tb3_move_cmd)


    # doesn't work really well currently
    # def Rotate(self):
    #     while True:
    #         self.x = self.destination[0] - self.current_position[0]
    #         self.y = self.destination[1] - self.current_position[1]

    #         self.angle_to_dest = atan2(self.y, self.x)

    #         if abs(self.angle_to_dest - self.rotation) > 0.005:
    #             self.tb3_move_cmd.linear.x = 0.0
    #             self.tb3_move_cmd.angular.z = 0.3
    #         else:
    #             self.tb3_move_cmd.angular.z = 0.0 
    #             self.cmd_vel.publish(self.tb3_move_cmd)
    #             break

    #         self.cmd_vel.publish(self.tb3_move_cmd)


    # callback function
    def callback_odom(self, msg):
        self.current_position[0], self.current_position[1] = msg.pose.pose.position.x, msg.pose.pose.position.y

        # rot_q = msg.pose.pose.orientation
        # (roll, pitch, self.rotation) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def callback_signal(self, msg):
        self.signal = msg.data

    def callback_system(self, msg):
        self.ready = msg.data
        
    def callback_other_odom(self, msg, args):
        self.same_direction_location_x[args] = round(msg.pose.pose.position.x, 2)
        self.same_direction_location_y[args] = round(msg.pose.pose.position.y, 2)

    def callback_prior_command(self, msg):
        self.prior_command = msg.data