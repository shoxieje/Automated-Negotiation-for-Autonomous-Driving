#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
import ActionType as types
from rocon_std_msgs.msg._StringArray import StringArray
from Directed_Graph import *

class IntersectionAgent:
    def __init__(self, total_robots):
        rospy.init_node('centercontrol', anonymous=True, disable_signals=True)
        self.total_robots = total_robots

        # we can have a function to calculate these value
        self.add_to_queue_distance = 4.0
        self.collision_region = 1.0
        self.safe_region = 0.5

        # initialize attribute related to vehicles
        self.pos_x = [0.0] * self.total_robots
        self.pos_y = [0.0] * self.total_robots
        # self.pos_z = [0.0] * self.total_robots
        self.direction = [''] * self.total_robots
        self.command = [None] * self.total_robots
        self.position = [None] * self.total_robots
        self.received_state = [None] * self.total_robots
        self.odom = [None] * self.total_robots
        self.cmd_vel = [None] * self.total_robots

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

        # position and speed of robots
        for i in range(self.total_robots):
            self.odom[i] = rospy.Subscriber('/tb3_' + str(i) + '/odom', Odometry, self.callback_odom, (i))
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

        while not rospy.is_shutdown():
            self.total.publish(self.direction)
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
                if abs(self.current_dist[self.active_queue[0]]) <= (self.collision_region + 0.05):
                    self.state[self.active_queue[0]] = types.ENTER_INTERSECTION
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
                    for i in self.active_queue[1:]:
                        if abs(self.current_dist[i]) <= self.collision_region:
                            self.state[i] = types.STOP
                            
                # check if the first vehicle from the queue has passed the threshold yet
                self.check_passed(self.active_queue[0])

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

        if abs(self.current_dist[first]) <= self.safe_region:
            self.state[first] = types.PASS_INTERSECTION
            self.active_queue.pop(0)
            
        # if self.direction[first] == types.DIR_LEFT or self.direction[first] == types.DIR_DOWN:
            # if self.current_dist[first] >= -self.safe_region:
                # self.state[first] = types.PASS_INTERSECTION
                # self.active_queue.pop(0)

        # elif self.direction[first] == types.DIR_LEFT or self.direction[first] == types.DIR_DOWN:
            # if self.current_dist[first] >= self.safe_region:
                # self.state[first] = types.PASS_INTERSECTION
                # self.active_queue.pop(0)

    def myhook(self):
        print("shutdown time!")

    def callback_odom(self, msg, args):
        self.pos_x[args] = msg.pose.pose.position.x
        self.pos_y[args] = msg.pose.pose.position.y
        # self.pos_z[args] = msg.pose.pose.position.z

    def callback_state(self, msg, args):
        self.state[args] = msg.data



if __name__ == "__main__":
    IntersectionAgent(total_robots=16)