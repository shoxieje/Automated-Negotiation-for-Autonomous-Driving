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

# create the files and make it executable
for i in range(num):
    f[i] = open("../First In First Out/Version 1/src/robots/FIFO_robot_{}.py".format(i), "w+")
    status[i] = os.stat("../First In First Out/Version 1/src/robots/FIFO_robot_{}.py".format(i))
    os.chmod("../First In First Out/Version 1/src/robots/FIFO_robot_{}.py".format(i), status[i].st_mode | stat.S_IEXEC)


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
