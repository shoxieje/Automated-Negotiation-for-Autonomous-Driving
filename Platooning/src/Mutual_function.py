#!/usr/bin/env python

import ActionType as types


def verticle_direction(direction):
    return direction == types.DIR_LEFT or direction == types.DIR_RIGHT or self.direction_with_turning()

def check_abs(x, y):
    return abs(x) > abs(y)


def get_distance(a, b):
    return abs(abs(a) - abs(b))
