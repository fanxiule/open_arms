#!/usr/bin/env python

import sys
import rospy
from open_arms_driver.srv import *
from open_arms_constants.open_arms_constants import JOINTS

def user_input():
    # prompt users to enter the joint and amount of rotation they want to move
    name_valid = False
    angle_valid = False

    while not name_valid:  # keep asking users for a valid input
        print("Enter the name of the joint which you want to move (e.g. joint1): ")
        joint_name = raw_input()
        if joint_name in JOINTS:
            name_valid = True
        else:
            print("Invalid joint name")

    while not angle_valid:  # keep asking users for a valid input
        print("How many degrees (+ve CCW, -ve CW) do you want the joint to rotate: ")
        try:
            joint_angle = float(raw_input())
            angle_valid = True
        except ValueError:
            print("Invalid angle")

    return joint_name, joint_angle


def deg_to_rev_client(name, angle):
    # client for deg_to_rev server which converts joint name and degree to format understood by Arduino
    rospy.wait_for_service('deg_to_rev')
    try:
        deg_to_rev = rospy.ServiceProxy('deg_to_rev', DegToRev)
        receipt = deg_to_rev(name, angle)
        return receipt.response
    except rospy.ServiceException:
        print "Joint name and angle not sent to Arduino"
        pass


if __name__ == '__main__':
    rospy.init_node('move_one_motor')
    while not rospy.is_shutdown():
        joint_name, joint_angle = user_input()
        response = deg_to_rev_client(joint_name, joint_angle)
        print(response)
