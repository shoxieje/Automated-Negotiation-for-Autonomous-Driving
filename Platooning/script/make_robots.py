#!/usr/bin/env python3
import sys
import os
import stat
import random

action_sys_path = sys.path[0][:-(len(sys.path[0]) - sys.path[0].rfind('/'))] + '/src/'
sys.path.insert(1, action_sys_path)

import ActionType as types

num = 0
if len(sys.argv) > 1:
    num = int(sys.argv[1])
else:
    num = int(input('How many robots do you want?\n'))

def randomize_path(i):
    while True:
        r = random.randint(0, 3)
        if r == 0 and i != 2:
            return types.DIR_RIGHT
        elif r == 1 and i != 0:
            return types.DIR_LEFT
        elif r == 2 and i != 3:
            return types.DIR_UP
        elif r == 3 and i != 1:
            return types.DIR_DOWN



f = [None] * num
status = [None] * num

desired_path = sys.path[0][:-(len(sys.path[0]) - sys.path[0].rfind('/'))] + '/src/robots_{}'.format(num)

if not os.path.exists(desired_path):
    os.makedirs(desired_path)

# create the files and make it executable
for i in range(num):
    f[i] = open(desired_path + "/FIFO_robot_{}_{}.py".format(i, num), "w+")
    status[i] = os.stat(desired_path + "/FIFO_robot_{}_{}.py".format(i, num))
    os.chmod(desired_path + "/FIFO_robot_{}_{}.py".format(i, num), status[i].st_mode | stat.S_IEXEC)

init_position = 4 + 0.5 * (num // 10) + 0.05 * (num % 10)
destination = 6 + num * 0.25


file_input = "#!/usr/bin/env python\nimport include_sys_path\nfrom FIFO_base_robot import *\n\n"

file_input += "if __name__ == \"__main__\":\n"

individual_input = [None] * num

# spawn left, down, right, up
pos_x_init = [-init_position, -0.25, init_position, 0.25]
pos_y_init = [0.25, -init_position, -0.25, init_position]

pos_x_destination = []
pos_y_destination = []

# left <-> right

for i in range(num):
    dir_path = randomize_path(i % 4)
    if dir_path == types.DIR_RIGHT:
        pos_x_destination.append(8.25)
        pos_y_destination.append(0.25)
    elif dir_path == types.DIR_LEFT:
        pos_x_destination.append(-8.25)
        pos_y_destination.append(-0.25)
    elif dir_path == types.DIR_DOWN:
        pos_x_destination.append(0.25)
        pos_y_destination.append(-8.25)
    else:
        pos_x_destination.append(-0.25)
        pos_y_destination.append(8.25)
    
    if i % 4 == 0:
        print('RIGHT_' + dir_path)
    elif i % 4 == 1:
        print('UP_' + dir_path)
    elif i % 4 == 2:
        print('LEFT_' + dir_path)
    else:
        print('DOWN_' + dir_path)



for i in range(num):
    if i >= 4:
        if i % 4 == 0:
            pos_x_init.append(pos_x_init[i - 4] - 0.5)
            pos_y_init.append(pos_y_init[0])
        elif i % 4 == 1:
            pos_x_init.append(pos_x_init[1])
            pos_y_init.append(pos_y_init[i - 4] - 0.5)
        elif i % 4 == 2:
            pos_x_init.append(pos_x_init[i - 4] + 0.5)
            pos_y_init.append(pos_y_init[2])
        else:
            pos_x_init.append(pos_x_init[3])
            pos_y_init.append(pos_y_init[i - 4] + 0.5)


    individual_input[i] = "\tRun_Node(name='{0}', init_position=[{1}, {2}], destination=[{3}, {4}])".format(i, pos_x_init[i], pos_y_init[i], pos_x_destination[i], pos_y_destination[i])



for i in range(num):
    f[i].write(file_input + individual_input[i])
    f[i].close()


t = open(desired_path + '/include_sys_path.py', "w+")

t_input = "import sys\nsys.path.append(sys.path[0][:-(len(sys.path[0]) - sys.path[0].rfind('/'))])"
t.write(t_input)
t.close()




