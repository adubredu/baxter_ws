#!/usr/bin/env python

import rospy
from run_scripts.srv import *
from std_msgs.msg import Bool


def handle_command(req):
    print "received command"
    turn = False
    if req.command in 'open the fridge yes':
        turn = True
    else:
        turn = False
    return CommandResponse(turn)

def add_two_ints_server():
    rospy.init_node('command_server')
    s = rospy.Service('receive_command', Command, handle_command)
    print "Ready to receive command."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()