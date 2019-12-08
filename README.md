# Automated Negotiation for Autonous Driving Project

## Project Description
Over the last decade, research on autonomous vehicles (AVs) has made revolutionary
progress, which brings us hope of safer, more convenient and efficient means of
transportation. An autonomous vehicle system is an integration of many technologies,
including computer vision, graphical processing, navigation, sensor technologies and so on.
Most significantly, the recent advance of machine learning technologies enables a selfdriving
car to learn to drive in any complex road situations with millions of accumulated
driving hours, which are way higher than any experienced human driver can reach.
However, driving is not a purely technical job but involves complicated social activities,
which could be hard to learn from experience. For instance, if two cars meet in a narrow
road or a long bridge on which only one car can go through, how do the cars decide which
one should reverse to give way to the other? Many of such a situation requires direct
interaction among vehicles, vehicles and infrastructures, or vehicles and authorities. Such
demands push the research on AVs to a different direction from machine learning with
regards to communication, negotiation and cooperation among autonomous vehicles.
Unfortunately, the studies along this direction is far from adequate. The primary goal of
this project is to design and implement automated negotiation protocols for autonomous
vehicles to negotiate each other when they meet unexpected road situations. The research
will be based on simulations using the well-known robot simulation system Gazebo. Each
robot acts as an autonomous vehicle and road traffic can then be simulated as a multi-robot
system. We will design a few specific unusual road situations, such as road with blocked
lines, junction without traffic control or totally blocked traffic jams. The task of the project
is to investigate negotiation protocols for robots to interact each other and resolve traffic
jams by themselves without external intervention. We will design and implement different
negotiation protocols to compare their effectiveness and efficiency.

## Project Aims
* Set up a multi-robot system under [Gazebo](http://gazebosim.org/) for testing automated negotiation
protocols
* Design and implement in Gazebo at least three specific road situations that can
likely lead to traffic jams
* Design and implement different automated negotiation protocols so that robots can
interact each other based on the protocols
* Design and implement negotiation strategies for robots to resolve traffic jams under
given negotiation protocols

## Project Methods
Description of negotiation protocols will be based on the concept of road graph, a novel
method we recently introduced for representing roads and traffic (published in PRICAI2019).
With the concept of road graph, we can then describe various traffic-related
elements, such as traffic flow, traffic-control protocols, vehicle information and vehicle
management processes. By using the facilities of Gazebo, we can implement a set of road
situations in 3D model. Robots can run in any of such a road situation. Programming of
robot will be based on ***Robot Operating System (ROS)*** running in Linux. Either C++ or
Python can be used. We prefer to use Python because it is simpler and more existing
libraries to use. For the testing traffic situations with only two vehicles involved, beside
simulations in Gazebo, we can also use TutleBot robots we currently have.

## Supported Version
This project is compatible with [***ROS Kinectic***](http://wiki.ros.org/kinetic) and [***Ubuntu 16.04***](http://releases.ubuntu.com/16.04/)