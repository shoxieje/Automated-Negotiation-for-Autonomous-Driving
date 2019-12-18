#!/usr/bin/env python3
import sys
import os

num = 0

if len(sys.argv) > 1:
    num = int(sys.argv[1])
else:
    num = int(input('How many robots do you want?\n'))


desired_path_launch = sys.path[0][:-(len(sys.path[0]) - sys.path[0].rfind('/'))] + '/launch/FIFO_launch_{}'.format(num)

if not os.path.exists(desired_path_launch):
    os.makedirs(desired_path_launch)


f = open('../launch/FIFO_launch_{0}/FIFO_robots_{0}.launch'.format(num), "w+")

file_input = "<launch>\n"

for i in range(num):
    file_input += "\t<node pkg=\"turtlebot3_gazebo\" type=\"FIFO_robot_{0}_{1}.py\" name=\"tb3_{0}\" output=\"screen\"></node>\n".format(i, num)


file_input += "</launch>"

f.write(file_input)
f.close()