#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from run_scripts.srv import *

def receive_client(command):
    print('waiting')
    rospy.wait_for_service('receive_command')
    try:
        print('yaay')
        parse_command = rospy.ServiceProxy('receive_command', Command)
        resp1 = parse_command(command)
        return resp1.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    command = 'open the fridge'
    print  receive_client(command)