#!/usr/bin/env python3
import os
import sys

num = 0
if len(sys.argv) > 1:
    num = int(sys.argv[1])
else:
    num = int(input('How many robots do you want?\n'))


os.system('./make_world.py ' + str(num))
os.system('./make_robots.py ' + str(num))
os.system('./make_launch_robots.py ' + str(num))
os.system('./make_centralize.py ' + str(num))

print('./make_world.py '+ str(num))
print('./make_robots.py '+ str(num))
print('./make_launch_robots.py '+ str(num))
print('./make_centralize.py '+ str(num))

