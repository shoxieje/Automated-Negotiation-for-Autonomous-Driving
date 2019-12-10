#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
import ActionType as types
from rocon_std_msgs.msg._StringArray import StringArray


class IntersectionAgent:
    def __init__(self, total_robots):
        rospy.init_node('centercontrol', anonymous=True)
        self.total_robots = total_robots

        # we can have a function to calculate these value
        self.add_to_queue_distance = 4.0
        self.collision_region = 2.0
        self.safe_region = 0.5

        # initialize attribute related to vehicles
        self.pos_x = [0.0] * self.total_robots
        self.pos_y = [0.0] * self.total_robots
        self.pos_z = [0.0] * self.total_robots
        self.speed = [0.0] * self.total_robots
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

        # position and speed of robots
        for i in range(self.total_robots):
            self.odom[i] = rospy.Subscriber('/tb3_' + str(i) + '/odom', Odometry, self.callback_odom, (i))
            self.cmd_vel[i] = rospy.Subscriber('/tb3_' + str(i) + '/cmd_vel', Twist, self.callback_cmd, (i))
            self.position[i] = rospy.wait_for_message('/tb3_' + str(i) + '/direction', String)
            self.direction[i] = self.position[i].data
            self.command[i] = rospy.Publisher('tb3_' + str(i) + '_command', String, queue_size=15)
            self.received_state[i] = rospy.Subscriber('tb3_' + str(i) + '/self_command', String, self.callback_state, (i))

        # store current distance
        self.current_dist = [0.0] * self.total_robots

        # check whether added to queue or not
        self.added_to_queue = [False] * self.total_robots

        # active queue represent the priority order
        self.active_queue = []
        

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
                self.state[self.active_queue[0]] = types.ENTER_INTERSECTION
                if self.active_queue[1:]:
                    for i in self.active_queue[1:]:
                        if abs(self.current_dist[i]) <= self.collision_region:
                            self.state[i] = types.STOP
                            
                # check if the first vehicle from the queue has passed the threshold yet
                self.is_passed(self.active_queue[0])

            # public commands to robots
            for i in range(self.total_robots):
                self.command[i].publish(self.state[i])

            print(self.state)
            # print(self.added_to_queue)
            self.rate.sleep()


    def is_passed(self, first):
        if abs(self.current_dist[first]) < self.safe_region:
            self.state[first] = types.PASS_INTERSECTION
            self.active_queue.pop(0)


    def callback_odom(self, msg, args):
        self.pos_x[args] = msg.pose.pose.position.x
        self.pos_y[args] = msg.pose.pose.position.y
        self.pos_z[args] = msg.pose.pose.position.z

    def callback_cmd(self, msg, args):
        self.speed[args] = msg.linear.x

    def callback_state(self, msg, args):
        self.state[args] = msg.data



if __name__ == "__main__":
    IntersectionAgent(total_robots=4)