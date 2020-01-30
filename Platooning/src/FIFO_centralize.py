#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
import ActionType as types
from rocon_std_msgs.msg._StringArray import StringArray

class IntersectionAgent:
    def __init__(self, total_robots):
        rospy.init_node('centercontrol', anonymous=True, disable_signals=True)
        self.total_robots = total_robots

        # we can have a function to calculate these value
        self.add_to_queue_distance = 3.1 + 0.5 * (self.total_robots // 10) + 0.05 * (self.total_robots % 10)
        print(self.add_to_queue_distance)
        self.collision_region = 1.0
        self.safe_region = 0.5

        # initialize attribute related to vehicles
        self.pos_x = [0.0] * self.total_robots
        self.pos_y = [0.0] * self.total_robots
        self.speed = [0.0] * self.total_robots

        self.direction = [''] * self.total_robots
        self.command = [None] * self.total_robots
        self.position = [None] * self.total_robots
        self.received_state = [None] * self.total_robots
        self.odom = [None] * self.total_robots
        self.cmd_vel = [None] * self.total_robots

        self.allocate_once = [False] * self.total_robots

        # init centralize
        rospy.loginfo('Center Control Available')
        self.rate = rospy.Rate(10)
        self.online = rospy.Publisher('online', String, queue_size=100)
        online_str = "Hello everyone, the Center Control Server is online."
        self.online.publish(online_str)

        self.ready = rospy.Publisher('ready', String, queue_size=100)
        self.ready.publish(types.NOT_READY)

        self.total = rospy.Publisher('total_direction', StringArray, queue_size=15)

        self.entered_once = False

        self.turning_distance_far = [0.0] * self.total_robots
        self.turning_distance_close = [0.0] * self.total_robots

        self.platoon_queue = []
        self.platoon_dict = {}
        self.sorted_platoon_dict = []
        self.enter_intersection = 0
        self.saved_sorted_platoon_dict = []

        self.opposite_dir = ''
        self.opp_dir_arr = []
        self.opp_pos_arr = []
        self.first_opp_vehicle = None
        self.min_pos_opp = 0.0
        self.min_once = True

        self.has_turned = [False] * self.total_robots
        self.first_direction = [''] * self.total_robots
        self.second_direction = [''] * self.total_robots

        self.turning_control = rospy.Publisher('turning_control', String, queue_size=15)

        # position and speed of robots
        for i in range(self.total_robots):
            # position and speed
            self.odom[i] = rospy.Subscriber('/tb3_' + str(i) + '/odom', Odometry, self.callback_odom, (i))
            self.cmd_vel = rospy.Subscriber('/tb3_' + str(i) + '/cmd_vel', Twist, self.callback_cmd, (i))
            self.position[i] = rospy.wait_for_message('/tb3_' + str(i) + '/direction', String)
            self.direction[i] = self.position[i].data
            self.command[i] = rospy.Publisher('tb3_' + str(i) + '_command', String, queue_size=15)
            self.received_state[i] = rospy.Subscriber('tb3_' + str(i) + '/self_command', String, self.callback_state, (i))
            if self.direction[i].count('_') == 1:
                self.first_direction[i] = self.direction[i][:self.direction[i].index('_')]
                self.second_direction[i] = self.direction[i][self.direction[i].index('_') + 1:]
        
        # store current distance
        self.current_dist = [0.0] * self.total_robots

        # active queue represent the priority order
        self.active_queue = []
        
        self.ready.publish(types.READY)

        self.added_to_queue = [False] * self.total_robots

        # represent the state of centralize model (init state is moving for all robots)
        self.state = [types.MOVING] * self.total_robots
        print(self.direction)

        self.added_to_speed = False

        while not rospy.is_shutdown():
            self.total.publish(self.direction)

            if not self.added_to_speed:
                if not 0.0 in self.speed:
                    self.added_to_speed = True


            for i in range(self.total_robots):
                # get the current distance to target
                self.current_dist[i] = self.pos_x[i] if self.verticle_direction(i) else self.pos_y[i]

                print("Robot {}: {} with {}".format(i, self.current_dist[i], self.direction[i]))

                # if a robot is just outside collision region -> add to queue
                if abs(self.current_dist[i]) <= self.add_to_queue_distance and not self.added_to_queue[i] and self.current_dist[i] != 0:
                    self.added_to_queue[i] = True
                    # active queue will hold the id of robots which serve FCFS
                    self.active_queue.append(i)

            print(self.active_queue)
            
            # handle queue
            if self.active_queue:
                if abs(self.current_dist[self.active_queue[0]]) <= (self.collision_region + 0.05) and not self.entered_once:
                    self.entered_once = True

                    self.state[self.active_queue[0]] = types.ENTER_INTERSECTION

                    # handle opposite direction
                    self.opposite_dir = self.detect_opposite_dir(self.direction[self.active_queue[0]])

                    # get the index of all opposite direction vehicles
                    self.opp_dir_arr = [i for i, x in enumerate(self.direction) if x == self.opposite_dir and (self.state[i] == types.MOVING or self.state[i] == types.STOP)]

                    for i in self.opp_dir_arr:
                        if self.min_pos_opp > abs(self.current_dist[i]) or self.min_once:
                            self.min_once = False
                            self.min_pos_opp = abs(self.current_dist[i])
                            self.first_opp_vehicle = i

                    if self.first_opp_vehicle is not None and self.check_turning_far(self.active_queue[0]) and self.check_turning_far(self.first_opp_vehicle):
                        self.t1 = self.calc_to_safe_distance(self.active_queue[0]) / self.speed[self.active_queue[0]]
                        self.t2 = self.calc_to_collision_distance(self.first_opp_vehicle) / self.speed[self.first_opp_vehicle]

                        if self.t1 >= self.t2:
                            self.state[self.first_opp_vehicle] = types.ENTER_INTERSECTION
                            self.active_queue.pop(self.active_queue.index(self.first_opp_vehicle))
                            self.active_queue.insert(1, self.first_opp_vehicle)
                            print(self.active_queue)
                    
                    self.first_opp_vehicle = None


                if self.active_queue[1:]:
                    self.platoon_dict = {}
                    self.platoon_queue = [i for i, x in enumerate(self.state) if x == types.PLATOONING]
                    print(self.platoon_queue)

                    for platoon_vehicle in self.platoon_queue:
                        self.platoon_dict[platoon_vehicle] = self.calc_to_safe_distance(platoon_vehicle) / self.speed[platoon_vehicle]

                    self.sorted_platoon_dict = sorted(self.platoon_dict.items(), key=lambda x: x[1])

                    if self.sorted_platoon_dict and self.sorted_platoon_dict[0][0] != self.active_queue[0]:
                        self.total_intersection = self.state.count(types.ENTER_INTERSECTION)
                    else:
                        self.total_intersection = 0

                    for i in self.sorted_platoon_dict:
                        value = i[0]
                        if len(self.saved_sorted_platoon_dict) < len(self.sorted_platoon_dict):
                            self.allocate_once[value] = False

                        if not self.allocate_once[value]:
                            self.allocate_once[value] = True
                            self.active_queue.remove(value)
                            self.active_queue.insert(self.total_intersection, value)
                            self.total_intersection += 1

                    print(self.sorted_platoon_dict)

                    for i in self.active_queue[1:]:
                        if abs(self.current_dist[i]) <= self.collision_region and self.state[i] != types.PLATOONING and self.state[i] != types.ENTER_INTERSECTION:
                            self.state[i] = types.STOP
                    
                    self.saved_sorted_platoon_dict = self.sorted_platoon_dict
                
                if len(self.active_queue) <= 2:
                    if (self.direction[self.active_queue[0]].count('_') == 1 and self.has_turned[self.active_queue[0]]) or self.direction[self.active_queue[0]].count('_') < 1:
                        self.check_passed(self.active_queue[0])
                else:
                    for i in range((self.state.count(types.ENTER_INTERSECTION)) + (1 if self.state.count(types.PLATOONING) >= 1 else 0)):
                        if (self.direction[self.active_queue[i]].count('_') == 1 and self.has_turned[self.active_queue[i]]) or self.direction[self.active_queue[i]].count('_') < 1:
                            self.check_passed(self.active_queue[i])


            # public commands to robots
            for i in range(self.total_robots):
                self.command[i].publish(self.state[i])
            

            print(self.state)

            if self.all_same(self.state):
                rospy.signal_shutdown('No Vehicles Available in the Intersection!')
                rospy.on_shutdown(self.myhook)
            else:
                self.rate.sleep()

    def all_same(self, x):
        return all(a == x[0] and a == types.PASS_INTERSECTION for a in x)

    def check_passed(self, first):
        if self.direction[first] == types.DIR_LEFT or self.direction[first] == types.DIR_DOWN or (self.check_second_direction(first) and self.has_turned[first]):
            if self.current_dist[first] <= -self.safe_region:
                self.state[first] = types.PASS_INTERSECTION
                self.active_queue.pop(self.active_queue.index(first))
                self.entered_once = False
                self.min_once = True
        else:
            if self.current_dist[first] >= self.safe_region:
                self.state[first] = types.PASS_INTERSECTION
                self.active_queue.pop(self.active_queue.index(first))
                self.entered_once = False
                self.min_once = True

    
    def check_second_direction(self, id):
        return self.second_direction[id] == types.DIR_LEFT or self.second_direction[id] == types.DIR_DOWN

    def myhook(self):
        print("shutdown time!")

    def detect_opposite_dir(self, x):
        return {
            types.DIR_DOWN: types.DIR_UP,
            types.DIR_UP: types.DIR_DOWN,
            types.DIR_LEFT: types.DIR_RIGHT,
            types.DIR_RIGHT: types.DIR_LEFT
        }.get(x[:x.rindex('_')] if x.count('_') == 1 else x, '')


    def calc_to_safe_distance(self, x):
        if self.direction[x] == types.DIR_LEFT or self.direction[x] == types.DIR_DOWN:
            return abs(abs(self.current_dist[x]) + self.safe_region)
        elif self.direction[x] == types.DIR_RIGHT or self.direction[x] == types.DIR_UP:
            return abs(self.current_dist[x] - self.safe_region)
        elif self.check_turning_far(x):
            return pow(abs(self.current_dist[x]), 2) + 0.5
        else:
            return pow(abs(self.current_dist[x]), 2) - 0.5


    def calc_to_collision_distance(self, x):
        return self.get_distance(self.current_dist[x], self.safe_region)

    def verticle_direction(self, id):
        if self.direction[id] == types.DIR_LEFT or self.direction[id] == types.DIR_RIGHT or self.direction_with_turning(id):
            return True
        return False

    def direction_with_turning(self, id):
        if self.direction[id].count('_') < 1:
            return False
        if self.calc_turned_direction(id) == types.LEFT or self.calc_turned_direction(id) == types.RIGHT:
            return True
        return False

    def calc_turned_direction(self, id):
        if self.has_turned[id]:
            return self.second_direction[id]

        if self.check_turn(id):
            self.has_turned[id] = True
            return self.second_direction[id]

        return self.first_direction[id]

    
    def check_turn(self, id):
        if self.first_direction[id] == types.DOWN:
            if self.second_direction[id] == types.LEFT:
                if self.pos_y[id] < -self.turning_distance_far[id]:
                    return True
            elif self.second_direction[id] == types.RIGHT:
                if self.pos_y[id] < self.turning_distance_close[id]:
                    return True

        elif self.first_direction[id] == types.UP:
            if self.second_direction[id] == types.RIGHT:
                if self.pos_y[id] > self.turning_distance_far[id]:
                    return True
            elif self.second_direction[id] == types.LEFT:
                if self.pos_y[id] > -self.turning_distance_close[id]:
                    return True

        elif self.first_direction[id] == types.LEFT:
            if self.second_direction[id] == types.UP:
                if self.pos_x[id] < -self.turning_distance_far[id]:
                    return True
            elif self.second_direction[id] == types.DOWN:
                if self.pos_x[id] < self.turning_distance_close[id]:
                    return True

        else:
            if self.second_direction[id] == types.DOWN:
                if self.pos_x[id] > self.turning_distance_far[id]:
                    return True
            elif self.second_direction[id] == types.UP:
                if self.pos_x[id] > -self.turning_distance_close[id]:
                    return True
        return False

    def check_turning_far(self, id):
        if self.first_direction[id] == types.RIGHT:
            return self.second_direction[id] == types.DOWN
        elif self.first_direction[id] == types.LEFT:
            return self.second_direction[id] == types.UP
        elif self.first_direction[id] == types.DOWN:
            return self.second_direction[id] == types.LEFT
        else:
            return self.second_direction[id] == types.RIGHT


    def get_distance(self, a, b):
        return abs(abs(a) - abs(b))


    def callback_odom(self, msg, args):
        self.pos_x[args] = msg.pose.pose.position.x
        self.pos_y[args] = msg.pose.pose.position.y
        # self.pos_z[args] = msg.pose.pose.position.z

    def callback_state(self, msg, args):
        self.state[args] = msg.data

    def callback_cmd(self, msg, args):
        if msg.linear.x != 0.0:
            self.speed[args] = msg.linear.x
            self.turning_distance_far[args] = 0.1 - (self.speed[args] - 0.1) * 1.5
            self.turning_distance_close[args] = 0.4 + (self.speed[args] - 0.1) * 1.5



if __name__ == "__main__":
    IntersectionAgent(total_robots=8)