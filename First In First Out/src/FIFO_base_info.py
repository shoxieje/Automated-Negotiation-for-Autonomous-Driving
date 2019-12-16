#!/usr/bin/env python
import ActionType as types

# initial data belongs to individual robots during the state
class Data(object):
    def __init__(self, name, initial_position, destination, signal="", rotation=0.0, speed=0.00, direction="", current_position=[0.0]*2):
        self.__name = name
        self.__destination = destination
        self.__signal = signal
        self.__rotation = rotation
        self.__initial_position = initial_position
        self.__speed = speed
        self.__direction = direction
        self.__current_position = current_position

########################################## signal

    @property
    def signal(self):
        return self.__signal
    
    @signal.setter
    def signal(self, value):
        self.__signal = value

########################################## rotation value

    @property
    def rotation(self):
        return self.__rotation
    
    @rotation.setter
    def rotation(self, value):
        self.__rotation = value

########################################## initial popsition
    
    @property
    def initial_position(self):
        return self.__initial_position
    
    @initial_position.setter
    def initial_position(self, value):
        self.__initial_position = value

########################################## speed

    @property
    def speed(self):
        return self.__speed

    @speed.setter
    def speed(self, value):
        self.__speed = value

########################################## name

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, value):
        self.__name = value

########################################## destination

    @property
    def destination(self):
        return self.__destination

    @destination.setter
    def destination(self, value):
        self.__destination = value


########################################## destination

    @property
    def direction(self):
        return self.__direction

    @direction.setter
    def direction(self, value):
        self.__direction = value


########################################## current position
    @property
    def current_position(self):
        return self.__current_position

    @current_position.setter
    def current_position(self, value):
        self.__current_position = value