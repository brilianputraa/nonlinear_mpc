#!/usr/bin/env python2.7

from cgi import test
import socket

import rospy
import numpy as np
import actionlib

from std_msgs.msg import Int32
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import PoseStamped


class BehaviorManager():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base_client', move_base_msgs.msg)
        self.PORT = rospy.get_param("/PORT_NUMBER")
        self.HOST = rospy.get_param("/SERVER_HOST")
        self.goal_lists_dir = rospy.get_param("/goal_lists_dir")


def main():
    bm = BehaviorManager()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((bm.HOST, bm.PORT))
        except:
            print("Error Connecting to the Server!")
        
        test_data = 2
        test_data = str(test_data).encode('utf8')
        s.sendall(test_data)

