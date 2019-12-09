#!/usr/bin/env python3
import sys

num = 0

if len(sys.argv) > 1:
    num = int(sys.argv[1])
else:
    num = int(input('How many robots do you want?\n'))

f = open('../First In First Out/Version 1/launch/FIFO_robots_{}.launch'.format(num), "w+")

file_input = "<launch>\n"

for i in range(num):
    file_input += "\t<node pkg=\"turtlebot3_gazebo\" type=\"FIFO_robot_{0}.py\" name=\"tb3_{0}\" output=\"screen\"></node>\n".format(i)


file_input += "</launch>"

f.write(file_input)
f.close()