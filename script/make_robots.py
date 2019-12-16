#!/usr/bin/env python3
import sys
import os
import stat

num = 0

if len(sys.argv) > 1:
    num = int(sys.argv[1])
else:
    num = int(input('How many robots do you want?\n'))

f = [None] * num
status = [None] * num

desired_path = sys.path[0][:-(len(sys.path[0]) - sys.path[0].rfind('/'))] + '/First In First Out/Version 2/src/robots_{}'.format(num)

if not os.path.exists(desired_path):
    os.makedirs(desired_path)

# create the files and make it executable
for i in range(num):
    f[i] = open(desired_path + "/FIFO_robot_{}_{}.py".format(i, num), "w+")
    status[i] = os.stat(desired_path + "/FIFO_robot_{}_{}.py".format(i, num))
    os.chmod(desired_path + "/FIFO_robot_{}_{}.py".format(i, num), status[i].st_mode | stat.S_IEXEC)


file_input = "#!/usr/bin/env python\nimport include_sys_path\nfrom FIFO_base_robot import *\n\n"

file_input += "if __name__ == \"__main__\":\n"

individual_input = [None] * num

pos_x_init = [-4.75, -0.25, 4.75, 0.25]
pos_y_init = [0.25, -4.75, -0.25, 4.75]

pos_x_destination = [8.25, -0.25, -8.25, 0.25]
pos_y_destination = [0.25, 8.25, -0.25, -8.25]


for i in range(num):
    if i >= 4:
        if i % 4 == 0:
            pos_x_init.append(pos_x_init[i - 4] - 0.5)
            pos_y_init.append(pos_y_init[0])
            pos_x_destination.append(pos_x_destination[i - 4] - 0.5)
            pos_y_destination.append(pos_y_destination[0])
        elif i % 4 == 1:
            pos_x_init.append(pos_x_init[1])
            pos_y_init.append(pos_y_init[i - 4] - 0.5)
            pos_x_destination.append(pos_x_destination[1])
            pos_y_destination.append(pos_y_destination[i - 4] - 0.5)
        elif i % 4 == 2:
            pos_x_init.append(pos_x_init[i - 4] + 0.5)
            pos_y_init.append(pos_y_init[2])
            pos_x_destination.append(pos_x_destination[i - 4] + 0.5)
            pos_y_destination.append(pos_y_destination[2])
        else:
            pos_x_init.append(pos_x_init[3])
            pos_y_init.append(pos_y_init[i - 4] + 0.5)
            pos_x_destination.append(pos_x_destination[3])
            pos_y_destination.append(pos_y_destination[i - 4] + 0.5)


    individual_input[i] = "\tRun_Node(name='{0}', init_position=[{1}, {2}], destination=[{3}, {4}])".format(i, pos_x_init[i], pos_y_init[i], pos_x_destination[i], pos_y_destination[i])



for i in range(num):
    f[i].write(file_input + individual_input[i])
    f[i].close()


t = open(desired_path + '/include_sys_path.py', "w+")

t_input = "import sys\nsys.path.append(sys.path[0][:-(len(sys.path[0]) - sys.path[0].rfind('/'))])"


t.write(t_input)
t.close()