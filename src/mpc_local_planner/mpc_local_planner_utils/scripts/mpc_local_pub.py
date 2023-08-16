#!/usr/bin/env python2.7
import re
import math
import sys
import rospy
import numpy as np
import pandas as pd
import tf

from ackermann_msgs.msg import AckermannDriveStamped
from mpc_local_planner_msgs.msg import OptimalControlResult
from actionlib_msgs.msg import GoalStatusArray


pub_ctrl = rospy.Publisher("/rbcar_robot_control/command", AckermannDriveStamped, queue_size=1)


# Global vars
x_data = []
y_data = []
yaw_data = []
vel_data = []


def ocp_results_cb(ocp_res):
    print(ocp_res)


def control_cb(goal_stats):
    goal = goal_stats.status_list
    
    if not goal:
        pass
    else:        
        goal = " ".join("{} ".format(item) for item in goal)        
        pattern = r"status:\s+(?P<number>\d+)"
        match = re.search(pattern, goal)
        
        if match:
            status = match.group("number")
            if status == "3":
                print("==================================== GOAL REACHED =============================================")
                # Halt control and align wheels
                control_msg = AckermannDriveStamped()
                control_msg.header.stamp = rospy.Time.now()
                control_msg.drive.steering_angle = 0.0
                control_msg.drive.acceleration = 0.0
                control_msg.drive.speed = 0.0
                control_msg.drive.jerk = 0.0
                
                pub_ctrl.publish(control_msg)
                
    
def main():
    sub_goal = rospy.Subscriber("/move_base/status", GoalStatusArray, control_cb, queue_size=1)
    sub_ctrl = rospy.Subscriber("/move_base/MpcLocalPlannerROS/ocp_result", OptimalControlResult, ocp_results_cb, queue_size=1)
    rospy.spin()
    pass
    

if __name__ == "__main__":
    rospy.init_node("mpc_local_pub_node", disable_signals=False)
    rate = rospy.Rate(20)
    try:
        main()
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
    