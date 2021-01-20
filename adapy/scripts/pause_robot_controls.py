#!/usr/bin/env python

import adapy
import openravepy
import rospy


# python 2/3 compliance
try:
    input = raw_input
except NameError:
    pass


if __name__ == "__main__":
    rospy.init_node("pauser", anonymous=True)
    openravepy.RaveInitialize(True)

    env, robot = adapy.initialize(sim=False)
    with adapy.util.pause_controls(robot):
        input("Robot control is paused; you can now move the robot with the built-in joystick. Press <enter> to return to previous control mode.")
        
