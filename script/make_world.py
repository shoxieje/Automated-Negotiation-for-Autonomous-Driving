#!/usr/bin/env python3
import sys

num = 0

if len(sys.argv) > 1:
    num = int(sys.argv[1])
else:
    num = int(input('How many robots do you want?\n'))


f = open("../First In First Out/Version 1/launch/FIFO_load_world_{}.launch".format(num), "w+")


file_input = "<launch>\n\t<arg name='model' default='$(env TURTLEBOT3_MODEL)' doc='model type [burger, waffle, waffle_pi]'/>\n\n\n"

pos_x = [-4.75, -0.25, 4.75, 0.25]
pos_y = [0.25, -4.75, -0.25, 4.75]
pos_yaw = [-0.1, 1.522, -3.15, -1.56]
pos_pitch = 0.0
pos_z = 0.0

for i in range(num):
    file_input += "\t<arg name='{0}_tb3' default='tb3_{0}' />\n".format(i)

file_input += "\n\n"

for i in range(num):

    if i >= 4:
        if i % 4 == 0:
            pos_x.append(pos_x[i - 4] - 0.5)
            pos_y.append(pos_y[0])
        elif i % 4 == 1:
            pos_x.append(pos_x[1])
            pos_y.append(pos_y[i - 4] - 0.5)
        elif i % 4 == 2:
            pos_x.append(pos_x[i - 4] + 0.5)
            pos_y.append(pos_y[2])
        else:
            pos_x.append(pos_x[3])
            pos_y.append(pos_y[i - 4] + 0.5)

    file_input += "\t<arg name='{0}_tb3_x_pos' default='{1}' />\n".format(i, pos_x[i])
    file_input += "\t<arg name='{0}_tb3_y_pos' default='{1}' />\n".format(i, pos_y[i])
    file_input += "\t<arg name='{0}_tb3_z_pos' default='{1}' />\n".format(i, pos_z)
    file_input += "\t<arg name='{0}_tb3_yaw' default='{1}' />\n".format(i, pos_yaw[i % 4])
    file_input += "\t<arg name='{0}_tb3_pitch' default='{1}' />\n".format(i, pos_pitch)
    file_input += "\n\n"

file_input += "\t<include file='$(find gazebo_ros)/launch/empty_world.launch'>\n"
file_input += "\t\t<arg name='world_name' value='$(find turtlebot3_gazebo)/worlds/intersection_wall.world' />\n"
file_input += "\t\t<arg name='paused' value='false' />\n"
file_input += "\t\t<arg name='use_sim_time' value='true' />\n"
file_input += "\t\t<arg name='gui' value='true' />\n"
file_input += "\t\t<arg name='headless' value='false' />\n"
file_input += "\t\t<arg name='debug' value='false' />\n"
file_input += "\t</include>"
file_input += "\n\n"

for i in range(num):
    file_input += "\t<group ns='$(arg {0}_tb3)'>\n".format(i)
    file_input += "\t\t<param name='robot_description' command='$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro' />\n"
    file_input += "\n\t\t<node pkg='robot_state_publisher' type='robot_state_publisher' name='robot_state_publisher' output='screen'>\n"
    file_input += "\t\t\t<param name='publish_frequency' type='double' value='50.0' />\n"
    file_input += "\t\t\t<param name='tf_prefix' value='$(arg {0}_tb3)' />\n".format(i)
    file_input += "\t\t</node>\n\n"
    file_input += "\t\t<node name='spawn_urdf' pkg='gazebo_ros' type='spawn_model' args='-urdf -model $(arg {0}_tb3) -x $(arg {0}_tb3_x_pos) -y $(arg {0}_tb3_y_pos) -z $(arg {0}_tb3_z_pos) -Y $(arg {0}_tb3_yaw) -P $(arg {0}_tb3_pitch) -param robot_description' />\n".format(i)
    file_input += "\t</group>\n"
    file_input += "\n"

file_input += "</launch>"

f.write(file_input)
f.close()