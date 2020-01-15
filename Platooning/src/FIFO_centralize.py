#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
import ActionType as types
from rocon_std_msgs.msg._StringArray import StringArray
from Directed_Graph import *
from openpyxl import Workbook, load_workbook

class IntersectionAgent:
    def __init__(self, total_robots):
        rospy.init_node('centercontrol', anonymous=True, disable_signals=True)
        self.total_robots = total_robots

        # workbook = load_workbook(filename="../catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/data_recorded.xlsx")
        # sheet = workbook.active

        # working with workbook
        # for i in range(self.total_robots):
        #     sheet["A{}".format(i + 2)] = i

        # sheet["B1"] = "Time"
        # sheet["C1"] = "Avg. Speed"
        # sheet["D1"] = "Initial Speed"
        # sheet["E1"] = "Travel Distance"

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

        self.vertices = []
        self.directed_graph = Graph()

        self.entered_once = False

        self.prev_platoon = None
        self.platoon_index = 0
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
        # position and speed of robots
        for i in range(self.total_robots):
            # position and speed
            self.odom[i] = rospy.Subscriber('/tb3_' + str(i) + '/odom', Odometry, self.callback_odom, (i))
            self.cmd_vel = rospy.Subscriber('/tb3_' + str(i) + '/cmd_vel', Twist, self.callback_cmd, (i))

            self.position[i] = rospy.wait_for_message('/tb3_' + str(i) + '/direction', String)
            self.direction[i] = self.position[i].data
            self.command[i] = rospy.Publisher('tb3_' + str(i) + '_command', String, queue_size=15)
            self.received_state[i] = rospy.Subscriber('tb3_' + str(i) + '/self_command', String, self.callback_state, (i))
            self.vertices.append(i)


        for i in self.vertices:
            self.directed_graph.add_vertex(Vertex(i))
        
        # store current distance
        self.current_dist = [0.0] * self.total_robots

        # check whether added to queue or not
        self.added_to_queue = [False] * self.total_robots
        self.added_to_graph = [False] * len(self.vertices)

        # active queue represent the priority order
        self.active_queue = []
        
        output_file = open("../catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/info.txt", "w+")

        self.ready.publish(types.READY)

        # represent the state of centralize model (init state is moving for all robots)
        self.state = [types.MOVING] * self.total_robots
        print(self.direction)

        self.added_to_speed = False

        while not rospy.is_shutdown():
            self.total.publish(self.direction)

            # add speed to worksheet once
            if not self.added_to_speed:
                if not 0.0 in self.speed:
                    self.added_to_speed = True
                    # for i in range(self.total_robots):
                    #     sheet["D{}".format(i + 2)] = self.speed[i]
                    # workbook.save(filename="../catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/data_recorded.xlsx")


            for i in range(self.total_robots):
                # get the current distance to target
                self.current_dist[i] = self.pos_x[i] if self.direction[i] == types.DIR_LEFT or self.direction[i] == types.DIR_RIGHT else self.pos_y[i]

                print("Robot {}: {} with {}".format(i, abs(self.current_dist[i]), self.direction[i]))

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

                    if self.first_opp_vehicle is not None:
                        self.t1 = self.calc_to_safe_distance(self.active_queue[0]) / self.speed[self.active_queue[0]]
                        self.t2 = self.calc_to_collision_distance(self.first_opp_vehicle) / self.speed[self.first_opp_vehicle]

                        if self.t1 >= self.t2:
                            self.state[self.first_opp_vehicle] = types.ENTER_INTERSECTION
                            self.active_queue.pop(self.active_queue.index(self.first_opp_vehicle))
                            self.active_queue.insert(1, self.first_opp_vehicle)
                            print(self.active_queue)
                    
                    self.first_opp_vehicle = None

                    if self.added_to_graph[self.active_queue[0]] == False:
                        self.added_to_graph[self.active_queue[0]] = True
                        for i in self.vertices:
                            if i != self.active_queue[0]:
                                self.directed_graph.add_edges(self.vertices[self.vertices.index(self.active_queue[0])], i)

                        self.vertices.pop(self.vertices.index(self.active_queue[0]))

                        output_file.write(self.directed_graph.print_graph() + '\n')

                        if len(self.vertices) == 0:
                            output_file.close()

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
                        if abs(self.current_dist[i]) <= self.collision_region and self.state[i] != types.PLATOONING and self.state[i] != types.MOVING_NEXT and self.state[i] != types.ENTER_INTERSECTION:
                            self.state[i] = types.STOP
                    
                    self.saved_sorted_platoon_dict = self.sorted_platoon_dict
                
                # check if the first vehicle from the queue has passed the threshold yet
                if len(self.active_queue) <= 2:
                    self.check_passed(self.active_queue[0])
                else:
                    for i in range(2):
                        self.check_passed(self.active_queue[i])

            # public commands to robots
            for i in range(self.total_robots):
                self.command[i].publish(self.state[i])

            print(self.state)

            if self.all_same(self.state):
                # workbook.save(filename="data_recorded.xlsx")
                rospy.signal_shutdown('No Vehicles Available in the Intersection!')
                rospy.on_shutdown(self.myhook)
            else:
                self.rate.sleep()

    def all_same(self, x):
        return all(a == x[0] and a == types.PASS_INTERSECTION for a in x)

    def check_passed(self, first):
        if self.direction[first] == types.DIR_LEFT or self.direction[first] == types.DIR_DOWN:
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

    def myhook(self):
        print("shutdown time!")

    def detect_opposite_dir(self, x):
        return {
            types.DIR_DOWN: types.DIR_UP,
            types.DIR_UP: types.DIR_DOWN,
            types.DIR_LEFT: types.DIR_RIGHT,
            types.DIR_RIGHT: types.DIR_LEFT
        }.get(x, '')


    def calc_to_safe_distance(self, x):
        if self.direction[x] == types.DIR_LEFT or self.direction[x] == types.DIR_DOWN:
            return abs(abs(self.current_dist[x]) + self.safe_region)
        else:
            return abs(self.current_dist[x] - self.safe_region)

    def calc_to_collision_distance(self, x):
        return self.get_distance(self.current_dist[x], self.safe_region)


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



if __name__ == "__main__":
    IntersectionAgent(total_robots=8)