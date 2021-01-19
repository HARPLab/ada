#!/usr/bin/env python

import adapy
import adapy.util

# python 2/3 compliance
try:
    input = raw_input
except NameError:
    pass


if __name__ == "__main__":
    env, robot = adapy.initialize()
    with adapy.util.pause_controls(robot):
        input("Robot control is paused; you can now move the robot with the built-in joystick. Press <enter> to return to previous control mode.")
        
