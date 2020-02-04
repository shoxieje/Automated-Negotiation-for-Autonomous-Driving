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
from openpyxl import Workbook, load_workbook

class Run_Node(Data):
    def __init__(self, name, init_position, destination):
        super(Run_Node, self).__init__(name, init_position, destination)

        # ros init
        rospy.init_node('centralize_tb3_' + self.name, anonymous=True)
        self.r = rospy.Rate(10)

        # should receive from the centralize
        self.safe_region = 0.5

        self.signal = ''
        self.ready = types.NOT_READY
        self.all_direction = ['']

        self.entered_once = True
        self.prior_speed = 0.0

        # speed publisher
        self.cmd_vel = rospy.Publisher('tb3_' + self.name +'/cmd_vel', Twist, queue_size=5)
        odom = rospy.Subscriber('tb3_' + self.name +'/odom', Odometry, self.callback_odom)
        sys_ready = rospy.Subscriber('ready', String, self.callback_system)

        self.same_direction = []

        self.safe_distance = 0.25

        # assign speed
        self.tb3_move_cmd = Twist()
        self.max_speed = 0.3
        # self.speed = round(random.uniform(0.1, 0.5), 2)

        # if self.speed > self.max_speed:
        #     self.speed = self.max_speed

        if self.name == '1':
            self.speed = 0.17
        elif self.name == '0' or self.name == '4' or self.name == '2' or self.name == '5' or self.name == '6' or self.name == '9':
            self.speed = 0.3
        elif self.name == '8':
            self.speed = 0.15
        elif self.name == '3':
            self.speed = 0.25
        elif self.name == '11':
            self.speed = 0.18
        elif self.name == '7':
            self.speed = 0.26
        else:
            self.speed = 0.22

# Robot 1: 0.17
# Robot 0: 0.3
# Robot 4: 0.3
# Robot 2: 0.3
# Robot 5: 0.3
# Robot 6: 0.3
# Robot 9: 0.3
# Robot 8: 0.15
# Robot 3: 0.25
# Robot 11: 0.18
# Robot 7: 0.26
# Robot 10: 0.22



        self.end_destination = 0.0

        self.check_first_direction_once = False

        self.first_direction_end_position = [0.0] * 2

        self.has_passed_intersection = False

        # self.speed = 0.15

        print('Robot {}: {}'.format(self.name, self.speed))

        # location publisher
        self.location = rospy.Publisher('tb3_' + self.name + '/direction', String, queue_size=5)
        self.direction = self.calc_direction()

        self.first_direction, self.second_direction = self.calc_separate_direction(self.direction)

        while self.ready == types.NOT_READY:
            self.location.publish(self.direction)

        # unsubscribe to system signal
        sys_ready.unregister()

        # get all the direction from other vehicles
        self.total_direction = rospy.wait_for_message('total_direction', StringArray)
        self.all_direction = self.total_direction.strings

        self.state = [''] * len(self.all_direction)

        state_from_centralize = rospy.Subscriber('state_control', StringArray, self.callback_state)

        # publish our own command
        self.self_command = rospy.Publisher('tb3_' + self.name + '/self_command', String, queue_size=5)

        # id of the vehicles in front
        self.prior_command = ''

        self.other_location = []
        self.prior_direction = ''
        self.prior_first_direction = ''
        self.prior_second_direction = ''
        
        self.turning_distance_far = self.calc_turning_distance_far()
        self.turning_distance_close = self.calc_turning_distance_close()
    
        if self.direction.count('_') == 1:
            self.has_turned = False
        else:
            self.has_turned = True

        # calculate same directions (based on the first direction)
        self.in_front = None
        self.in_front = self.calc_infront_vehicle()

        print('Robot {} in front {}'.format(self.name, self.in_front))

        self.unsubscribe_same_direction()

        self.road_distance = 5.0 + len(self.same_direction) * 0.5

        # method to get the front vehicle id

        # subscribe to the speed of the vehicle in front
        if self.in_front is not None:
            self.prior_vehicle_command = rospy.Subscriber('/tb3_' + str(self.in_front) + '_command', String, self.callback_prior_command)
            self.prior_vehicle_speed = rospy.Subscriber('tb3_' + str(self.in_front) + '/cmd_vel', Twist, self.callback_prior_speed)
            self.prior_direction = self.all_direction[self.in_front]

            self.prior_first_direction, self.prior_second_direction = self.calc_separate_direction(self.prior_direction)
            
        self.t1 = 0.0
        self.t2 = 0.0

        rospy.sleep(1)

        self.passed_intersection_once = False

        signal_data = rospy.Subscriber('tb3_'+ self.name +'_command', String, self.callback_signal)

        self.in_front_moved = True

        while not rospy.is_shutdown():
            # guide the robot toward the direction
            self.rotate()
            self.position = self.current_position[0] if self.verticle_direction() else self.current_position[1]

            # adjust the speed when it's get too close to the front vehicle
            if self.in_front is not None and (self.prior_command == types.MOVING or self.prior_command == types.PASS_INTERSECTION or self.prior_command == types.REACH_DESTINATION):
                # get the front vehicle position
                if self.verticle_direction():
                    self.prior_position = self.same_direction_location_x[self.same_direction.index(self.in_front)]
                else:
                    self.prior_position = self.same_direction_location_y[self.same_direction.index(self.in_front)]

                if self.get_distance(self.position, self.prior_position) <= (self.safe_distance + 0.1):
                    # make sure the prior speed doesn't change
                    if self.speed > self.prior_speed and self.prior_speed != 0.0:
                        self.speed = self.prior_speed
                        self.turning_distance_far = self.calc_turning_distance_far()
                        self.turning_distance_close = self.calc_turning_distance_close()

            # handle multiple signal coming from the front vehicle
            if self.prior_command == types.STOP:
                self.in_front_moved = True
                if self.get_distance(self.prior_position, self.position) <= self.safe_distance:
                     # stop immendiately
                    self.publish_signal(types.STOP)

            elif (self.prior_command == types.ENTER_INTERSECTION or self.prior_command == types.PLATOONING) and self.entered_once:

                self.entered_once = False
                # start moving when the other vehicles enter the area

                self.t1 = self.calc_to_safe_distance() / self.speed
                self.t2 = self.calc_to_collision_distance() / self.speed

                if self.t1 >= self.t2 and self.state.count(types.ENTER_INTERSECTION) <= 1:
                    self.publish_signal(types.PLATOONING)
                else:
                    self.publish_signal(types.MOVING)


            elif self.prior_command == types.MOVING and self.in_front_moved:
                self.in_front_moved = False
                self.publish_signal(types.MOVING)

            elif self.prior_command == types.REACH_DESTINATION and self.signal == types.PASS_INTERSECTION:
                # if self.name == '5':
                #     print(self.position, self.in_front, self.prior_position, self.verticle_direction(), self.same_direction_location_x, self.same_direction_location_y)
                if self.get_distance(self.position, self.prior_position) <= (self.safe_distance + 0.1):
                    self.publish_signal(types.REACH_DESTINATION)


            # * choose_status is working
            self.choose_status(self.signal)

            # * Shutdown the node when it reaches the destination
            if self.reached_destination():
                self.publish_speed_signal(0.0)
                self.publish_signal(types.REACH_DESTINATION)
                rospy.signal_shutdown('Passed Intersection!')
                rospy.on_shutdown(self.myhook)


    def choose_status(self, x):
        return {
            types.MOVING: self.Start,
            types.ENTER_INTERSECTION: self.Enter_intersection,
            types.PASS_INTERSECTION: self.Pass_intersection,
            types.PLATOONING: self.Platooning,
            types.STOP: self.Stop,
            types.REACH_DESTINATION: self.Reach_destination
        }.get(x, self.Stop)()


    # calculate the direction
    def calc_direction(self):
        if self.initial_position[1] == self.destination[1]:
            if self.initial_position[0] < 0:
                return types.DIR_RIGHT
            else:
                return types.DIR_LEFT
        elif self.initial_position[0] == self.destination[0]:
            if self.initial_position[1] < 0:
                return types.DIR_UP
            else:
                return types.DIR_DOWN

        else:
            if abs(self.initial_position[0]) < abs(self.initial_position[1]):
                # up <-> down
                if self.initial_position[1] > 0:
                    if self.destination[0] < 0:
                        return types.DIR_DOWN_LEFT
                    else:
                        return types.DIR_DOWN_RIGHT

                else:
                    if self.destination[0] < 0:
                        return types.DIR_UP_LEFT
                    else:
                        return types.DIR_UP_RIGHT

            else:
                # left <-> right
                if self.initial_position[0] > 0:
                    if self.destination[1] < 0:
                        return types.DIR_LEFT_DOWN
                    else:
                        return types.DIR_LEFT_UP

                else:
                    if self.destination[1] < 0:
                        return types.DIR_RIGHT_DOWN
                    else:
                        return types.DIR_RIGHT_UP


####################### calculate the distance
    def get_distance(self, a, b):
        return abs(abs(a) - abs(b))

    def find_closest_distance(self, l):

        if not self.has_passed_intersection:
            t = self.initial_position[0] if self.verticle_direction() else self.initial_position[1]
            closest_values = [x for num, x in enumerate(l) if abs(x) < abs(t) and (not self.has_turned or self.other_crossed_intersection[num] or self.first_direction == self.second_direction)]
        else:
            t = self.position
            closest_values = [x for num, x in enumerate(l) if abs(x) > abs(t) and (not self.has_turned or self.other_crossed_intersection[num] or self.first_direction == self.second_direction)]
        
        if closest_values:
            if not self.has_passed_intersection:
                return max(closest_values) if t > 0 else min(closest_values)
            else:
                return min(closest_values) if t > 0 else max(closest_values)
                
        return 0

    def calc_turning_distance_far(self):
        return (0.1 - (self.speed - 0.1) * 1.5)

    def calc_turning_distance_close(self):
        return (0.4 + (self.speed - 0.1) * 1.5)

    def calc_to_collision_distance(self):
        return self.get_distance(self.position, self.safe_region)

    def calc_to_safe_distance(self):
        if self.prior_direction == types.DIR_LEFT or self.prior_direction == types.DIR_DOWN:
            return abs(abs(self.prior_position) + self.safe_region)
        elif self.prior_direction == types.DIR_RIGHT or self.prior_direction == types.DIR_UP:
            return abs(self.prior_position - self.safe_region)
        elif self.check_turning_far():
            return pow(abs(self.prior_position), 2) + 0.5  #? c^2 = a^2 + b^2 (c = hypotenuse of the triangle, a = current distance - 0.25, b = 0.25^2)
        else:
            return pow(abs(self.prior_position), 2) - 0.5


    def check_turning_far(self):
        if self.prior_first_direction == types.DIR_RIGHT:
            return self.prior_second_direction == types.DIR_DOWN
        elif self.prior_first_direction == types.DIR_LEFT:
            return self.prior_second_direction == types.DIR_UP
        elif self.prior_first_direction == types.DIR_DOWN:
            return self.prior_second_direction == types.DIR_LEFT
        else:
            return self.prior_second_direction == types.DIR_RIGHT


    def calc_separate_direction(self, full_direction):
        if full_direction.count('_') == 1:
            return full_direction[:full_direction.index('_')], full_direction[full_direction.index('_') + 1:]
        return full_direction, full_direction

##################################### Moving function

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

        if not self.has_passed_intersection:
            self.has_passed_intersection = True
        
        if self.has_passed_intersection and not self.passed_intersection_once:
            self.passed_intersection_once = True

            if self.in_front is not None:
                self.prior_vehicle_command.unregister()
                self.prior_vehicle_speed.unregister()

            self.in_front = self.calc_infront_vehicle()

            print('-' * 50)
            print('Robot {} in front {}'.format(self.name, self.in_front))
            
            if self.in_front is not None:
                self.prior_vehicle_command_2 = rospy.Subscriber('/tb3_' + str(self.in_front) + '_command', String, self.callback_prior_command)
                self.prior_vehicle_speed_2 = rospy.Subscriber('tb3_' + str(self.in_front) + '/cmd_vel', Twist, self.callback_prior_speed)

            # reset prior command
            self.prior_command = ''
 

    def Platooning(self):
        # assign random speed (could add acceleration speed later)
        self.publish_speed_signal(self.speed)

    def Reach_destination(self):
        self.publish_speed_signal(0.0)


    def rotate(self):
        if not self.check_first_direction_once:
            self.check_first_direction_once = True
            if abs(self.initial_position[0]) > abs(self.initial_position[1]):
                if self.initial_position[0] < 0:
                    self.first_direction_end_position = [self.initial_position[0] + self.road_distance, self.initial_position[1]]
                else:
                    self.first_direction_end_position = [self.initial_position[0] - self.road_distance, self.initial_position[1]]
            else:
                if self.initial_position[1] < 0:
                    self.first_direction_end_position = [self.initial_position[0], self.initial_position[1] + self.road_distance]
                else:
                    self.first_direction_end_position = [self.initial_position[0], self.initial_position[1] - self.road_distance]


        if not self.has_turned:
            path_angle = atan2(self.first_direction_end_position[1] - self.current_position[1], self.first_direction_end_position[0] - self.current_position[0])
        else:
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
        if self.signal == types.REACH_DESTINATION:
            return True

        if self.signal == types.PASS_INTERSECTION:
            self.end_destination = self.destination[0] if self.verticle_direction() else self.destination[1]

            if abs(self.position) > (abs(self.end_destination) - 0.5):
                return True

        return False

    def verticle_direction(self):
        if self.direction == types.DIR_LEFT or self.direction == types.DIR_RIGHT or self.direction_with_turning():
            return True
        return False

    def direction_with_turning(self):
        if self.direction.count('_') < 1:
            return False
        if self.calc_turned_direction() == types.DIR_LEFT or self.calc_turned_direction() == types.DIR_RIGHT:
            return True
        return False

    def calc_turned_direction(self):
        if self.has_turned:
            return self.second_direction

        if self.check_turn(self.first_direction, self.second_direction, self.current_position):
            self.has_turned = True
            return self.second_direction

        return self.first_direction

    
    def check_turn(self, first_dir, second_dir, current_pos):
        if first_dir == types.DIR_DOWN:
            if second_dir == types.DIR_LEFT or second_dir == types.DIR_DOWN:
                return current_pos[1] < -self.turning_distance_far
            elif second_dir == types.DIR_RIGHT:
                return current_pos[1] < self.turning_distance_close

        elif first_dir == types.DIR_UP:
            if second_dir == types.DIR_RIGHT or second_dir == types.DIR_UP:
                return current_pos[1] > self.turning_distance_far
            elif second_dir == types.DIR_LEFT:
                return current_pos[1] > -self.turning_distance_close

        elif first_dir == types.DIR_LEFT:
            if second_dir == types.DIR_UP or second_dir == types.DIR_LEFT:
                return current_pos[0] < -self.turning_distance_far
            elif second_dir == types.DIR_DOWN:
                return current_pos[0] < self.turning_distance_close

        else:
            if second_dir == types.DIR_DOWN or second_dir == types.DIR_RIGHT:
                return current_pos[0] > self.turning_distance_far
            elif second_dir == types.DIR_UP:
                return current_pos[0] > -self.turning_distance_close


    def calc_infront_vehicle(self):
        if self.in_front is not None:
            self.other_location[self.same_direction.index(self.in_front)].unregister()

        self.same_direction = []
        self.other_location = []
        self.other_location_after_passed = []

        for i in range(len(self.all_direction)):
            if not self.has_passed_intersection:
                if self.all_direction[i][:len(self.first_direction)] == self.first_direction[:len(self.all_direction[i])] and str(i) != self.name:
                    self.same_direction.append(i)
            else:
                if self.all_direction[i].count('_') == 1:
                    if self.all_direction[i][self.all_direction[i].index('_') + 1:] == self.second_direction and str(i) != self.name:
                        self.same_direction.append(i)
                else:
                    if self.all_direction[i] == self.second_direction and str(i) != self.name:
                        self.same_direction.append(i)

        self.other_crossed_intersection = [False] * len(self.same_direction)
        self.same_direction_location_x = [0.0] * len(self.same_direction)
        self.same_direction_location_y = [0.0] * len(self.same_direction)
            
        if not self.has_passed_intersection:
            for i in self.same_direction:
                self.other_location.append(rospy.Subscriber('/tb3_' + str(i) + '/odom', Odometry, self.callback_other_odom, (self.same_direction.index(i))))
        else:
            for i in self.same_direction:
                self.other_location_after_passed.append(rospy.Subscriber('/tb3_' + str(i) + '/odom', Odometry, self.callback_other_odom, (self.same_direction.index(i))))

        # wait for the signal
        while 0.0 in self.same_direction_location_x and 0.0 in self.same_direction_location_y:
            pass

        if self.has_passed_intersection:
            for i, x in enumerate(self.same_direction):
                temp_dir_1, temp_dir_2 = self.calc_separate_direction(self.all_direction[x])
                self.other_crossed_intersection[i] = self.check_turn(temp_dir_1, temp_dir_2, [self.same_direction_location_x[i], self.same_direction_location_y[i]])


        if self.verticle_direction():
            self.closest_x = self.find_closest_distance(self.same_direction_location_x)
            if self.closest_x != 0:
                return self.same_direction[self.same_direction_location_x.index(self.closest_x)]

        else:
            self.closest_y = self.find_closest_distance(self.same_direction_location_y)
            if self.closest_y != 0:
                return self.same_direction[self.same_direction_location_y.index(self.closest_y)]

        return None

    def unsubscribe_same_direction(self):
        # unsubscribe to unnecessary topic
        for i in self.same_direction:
            if i != self.in_front:
                self.other_location[self.same_direction.index(i)].unregister()

    def calc_other_crossed_intersection(self):
        for i, x in enumerate(self.same_direction):
            temp_dir_1, temp_dir_2 = self.calc_separate_direction(self.all_direction[x])
            if temp_dir_1 == types.DIR_LEFT:
                if self.same_direction_location_x[i] < 0.45:
                    self.other_crossed_intersection[i] = True
            elif temp_dir_1 == types.DIR_RIGHT:
                if self.same_direction_location_x[i] > -0.45:
                    self.other_crossed_intersection[i] = True
            elif temp_dir_1 == types.DIR_DOWN:
                if self.same_direction_location_y[i] < 0.45:
                    self.other_crossed_intersection[i] = True
            else:
                if self.same_direction_location_y[i] > -0.45:
                    self.other_crossed_intersection[i] = True

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

    def callback_state(self, msg):
        self.state = msg.strings