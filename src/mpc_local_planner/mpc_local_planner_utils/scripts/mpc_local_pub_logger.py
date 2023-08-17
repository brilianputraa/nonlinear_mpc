#!/usr/bin/env python2.7
import datetime
import re
import signal
import math
import sys
import traceback
import os

import rospy
import numpy as np
import pandas as pd
import tf
import message_filters

from ackermann_msgs.msg import AckermannDriveStamped
from mpc_local_planner_msgs.msg import OptimalControlResult
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray

class ControlLogger:
    
    def __init__(self):
        self.odom_x = []
        self.odom_y = []
        self.odom_yaw = []
        self.odom_vel = []
        self.mpc_vel = []
        self.mpc_steer = []
        self.N = rospy.get_param("/move_base/MpcLocalPlannerROS/grid/grid_size_ref")
        self.pub_ctrl = rospy.Publisher("/rbcar_robot_control/command", AckermannDriveStamped, queue_size=1)
        self.sub_goal = rospy.Subscriber("/move_base/status", GoalStatusArray, self.control_cb, queue_size=1)
        self.sub_ctrl = message_filters.Subscriber("/move_base/MpcLocalPlannerROS/ocp_result", OptimalControlResult)
        self.sub_odom =  message_filters.Subscriber("/odoms", Odometry)
        
        self.header_measurement = ["odom_x", "odom_y", "odom_vel", "odom_yaw"]
        self.header_mpcOutput = ["mpc_vel", "mpc_steer"]
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_ctrl, self.sub_odom], 10, 0.1, allow_headerless=True)
        signal.signal(signal.SIGINT, self.sigint_handler)
        
    def control_cb(self, goal_stats):
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
                    
                    self.pub_ctrl.publish(control_msg)
        
    def save_data(self):
        dataFrame = {}

        data_measurement = [self.odom_x, self.odom_y, self.odom_vel, self.odom_yaw]
        data_mpcOutput = [self.mpc_vel, self.mpc_steer]

        for i in range(len(self.header_measurement)):
            dataFrame[self.header_measurement[i]] = data_measurement[i][:]

        for i in range(len(self.header_mpcOutput)):
            dataFrame[self.header_mpcOutput[i]] = data_mpcOutput[i][:]

        cur_date = datetime.datetime.now()
        cur_date = cur_date.strftime("%Y%m%d%H%M")
        df = pd.DataFrame(dataFrame)
        # print(df)
        # print(os.getcwd())
        df.to_csv('./src/mpc_local_planner/mpc_local_planner_utils/scripts/logs/%s_mpc_local_planner.csv' % cur_date)
                
    def sigint_handler(self, signal, frame):
        print('KeyboardInterrupt is caught')
        self.save_data()
        print("Logging data saved!")
        sys.exit(0)
            
    def data_logger_cb(self, ctrl, odom):
        try:
            quaternion = (
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w
            )

            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y

            # Obtain reference path
            euler = tf.transformations.euler_from_quaternion(quaternion)
            psi = euler[2]
            
            v = math.sqrt(odom.twist.twist.linear.x ** 2 + odom.twist.twist.linear.y ** 2)

            # Determine the car whether moving forward or backward using the heading and the velocity vector
            if np.dot([math.cos(psi), math.sin(psi)], [odom.twist.twist.linear.x, odom.twist.twist.linear.y]) > 0:
                v *= 1
            else:
                v *= -1
                
            self.odom_x.append(x)
            self.odom_y.append(y)
            self.odom_yaw.append(psi)
            self.odom_vel.append(v)
            
            speed = ctrl.controls[0]
            steer = ctrl.controls[1]
            self.mpc_vel.append(speed)
            self.mpc_steer.append(steer)
            
        except:
            print(traceback.format_exc())
            print("\nSave data and exit")
            self.save_data()
            sys.exit(1)
            
def main():
    rospy.init_node('control_logger', anonymous=True)
    obj = ControlLogger()
    obj.ts.registerCallback(obj.data_logger_cb)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()