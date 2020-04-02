#!/usr/bin/env python

import sys
import rospy
from open_arms_driver.srv import DegToRev, DegToRevResponse
from open_arms_driver.srv import *
from open_arms_constants.open_arms_constants import JOINT7, MOTOR_REDUCTION

deg = 0
name = ""


def deg_conversion(deg, name):
    if deg >= 0:
        # if deg > 0, the joint rotates CCW
        # hence the motor rotates CW
        # if deg < 0, the joint rotates CW
        # hence the motor rotates CCW
        # For void Stepper::moveRev(bool direction, float revolution) in OpenArmsStepper.h,
        # direction = true corresponds to CCW rotation of the motor
        # direction = false corresponds to CW rotation of the motor
        direction = False
    else:
        direction = True
    if name == JOINT7:
        revolution = abs(deg)/360.0
    else:
        revolution = abs(deg)/360.0*MOTOR_REDUCTION
    return direction, revolution


def move_motor_arduino_client(name, dir, rev):
    # client for sending joint name, direction and # of revolutions to Arduino
    rospy.wait_for_service('move_motor_arduino')
    try:
        move_motor_arduino = rospy.ServiceProxy(
            'move_motor_arduino', MoveMotorArduino)
        receipt = move_motor_arduino(name, dir, rev)
        return receipt.response
    except rospy.ServiceException:
        print "Joint name, direction and # of revolutions not sent to Arduino"
        pass


def handle_deg_to_rev(req):
    response = req.name + " and " + \
        str(req.deg) + " deg processed"
    dir, rev = deg_conversion(req.deg, req.name)
    res_arduino = move_motor_arduino_client(req.name, dir, rev)
    print(res_arduino)
    return DegToRevResponse(response)


def deg_to_rev_server():
    # receive joint name and angle from the client then convert it to info understood by Arduino
    degtorev_s = rospy.Service('deg_to_rev', DegToRev, handle_deg_to_rev)


if __name__ == "__main__":
    rospy.init_node('deg_to_rev_relay')
    while not rospy.is_shutdown():
        deg_to_rev_server()
        rospy.spin()
