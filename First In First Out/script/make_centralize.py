#!/usr/bin/env python3
import os
import sys
import stat

num = 0
if len(sys.argv) > 1:
    num = int(sys.argv[1])
else:
    num = int(input('How many robots do you want?\n'))

desired_path_launch = sys.path[0][:-(len(sys.path[0]) - sys.path[0].rfind('/'))] + '/launch/FIFO_launch_{}'.format(num)

if not os.path.exists(desired_path_launch):
    os.makedirs(desired_path_launch)


f = open('../launch/FIFO_launch_{0}/FIFO_centralize_{0}.launch'.format(num), "w+")

file_input = "<launch>\n"
file_input += "\t<node pkg='turtlebot3_gazebo' type='FIFO_centralize_{}.py' name='centercontrol' output='screen'></node>".format(num)
file_input += "\n</launch>"

f.write(file_input)
f.close()

src = open('../src/FIFO_centralize.py', 'rt')
dst = open('../src/FIFO_centralize_{}.py'.format(num), 'wt')

for line in src:
    dst.write(line.replace('IntersectionAgent(total_robots=8)', 'IntersectionAgent(total_robots={})'.format(num)))

status = os.stat("../src/FIFO_centralize_{}.py".format(num))
os.chmod("../src/FIFO_centralize_{}.py".format(num), status.st_mode | stat.S_IEXEC)

src.close()
dst.close()