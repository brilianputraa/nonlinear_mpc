#!/usr/bin/env python2.7

import math
import os
import datetime
import rospy
import actionlib
import roslaunch
import numpy as np
import tf.transformations
import GazeboHelper as gh
import GoalCheckerandGetter as gcg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal


class BatchSim:
    def __init__(self):
        # Pose and Twist of the robot
        self.current_pose = []
        self.current_pose_x = None
        self.current_pose_y = None
        self.current_pose_heading = None
        self.current_twist_linear_x = None
        self.current_twist_linear_y = None
        self.current_twist_angular_x = None
        self.current_twist_angular_y = None
        self.current_twist_angular_z = None
        self.previous_pose_x = None
        self.previous_pose_y = None
        self.previous_command_vel = None
        self.previous_command_steering = None
        self.odom_stamp = None
        self.previous_odom_stamp = None
        # Control commands
        self.current_vel_command = None
        self.current_steering_command = None
        # Time measurement and distance measurement
        self.distance_travelled = 0
        self.time_elapsed = 0
        # Goal status
        self.goal_status = None
        self.goal_reached = False
        self.status = None
        self.timeout = False
        self.time = None
        # Helper classes
        self.gh = gh.GazeboHelper()
        self.gcg = gcg.GoalCheckerandGetter()
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Set the initial pose of the robot in the gazebo world
        self.pose_z = 0.098094
        # Get the rosparam
        self.pickup_point_list_dir = rospy.get_param("~pickup_point_list_dir")
        self.mpc_local_planner_launch_dir = rospy.get_param("~mpc_without_global_planner_dir")
        # ROS Subscribers
        self.odom_sub = rospy.Subscriber("/INS/odom", Odometry, self.odom_callback)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        # Get the goal pose of the robot in the gazebo world
        self.pickup_point_list = np.loadtxt(self.pickup_point_list_dir, delimiter=',')
        # Grid area of the search
        self.grid_x_min = 26.071
        self.grid_x_max = 29.071
        self.grid_y_min = -1.609
        self.grid_y_max = 1.391
        self.grid_yaw_max = 0.0
        self.grid_yaw_min = -np.pi/4
        # Incrementing grid size for the grid search
        self.increment_grid_size_x = 0.2
        self.increment_grid_size_y = 0.2
        self.increment_grid_size_yaw = 0.0872665  
        # ROS Timeouts
        self.move_base_client_timeout = 50.0
        # Loggers
        self.current_date_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.logdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "logs", self.current_date_time)
        self.log_file_everything = None
        self.log_file_per_grid_point = None
        self.log_file_name_grid_point = None

    def run_batch_sim(self):
        os.mkdir(self.logdir)
        rospy.sleep(0.1)
        with open(os.path.join(self.logdir, "log_all_grid.txt"), "w") as self.log_file_everything:
            # Write Header to the log file
            # print("WRITING HEADER")
            self.log_file_everything.write("x_grid,y_grid,yaw_grid,distance_travelled,time_elapsed,goal_status\n")
            # For each x in the grid area
            for x in np.arange(self.grid_x_min, self.grid_x_max, self.increment_grid_size_x):
                # For each y in the grid area
                for y in np.arange(self.grid_y_min, self.grid_y_max, self.increment_grid_size_y):
                    # For each yaw in the grid area
                    for yaw in np.arange(self.grid_yaw_min, self.grid_yaw_max, self.increment_grid_size_yaw):
                        # Set the log file name for the current grid point
                        self.log_file_name_grid_point = "log_per_grid_point_" + str(x) + "_" + str(y) + "_" + str(yaw) + ".txt"
                        # Open the log file for the current grid point
                        with open(os.path.join(self.logdir, self.log_file_name_grid_point), "w") as self.log_file_per_grid_point:
                            # Write Header to the log file
                             # print("WRITING HEADER")
                            self.log_file_per_grid_point.write("time,current_pose_x,current_pose_y,current_pose_heading,current_twist_linear_x,current_twist_linear_y,current_twist_angular_x,current_twist_angular_y,current_twist_angular_z,current_vel_command,current_steering_command,goal_status\n")
                            # Set the pose of the robot to the current grid poin
                            self.gh.setModelPoseAndSpeed("rbcar", [x, y, self.pose_z, 0.0, 0.0, yaw], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                            rospy.sleep(0.1)
                            # Launch the mpc_local_planner
                            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                            roslaunch.configure_logging(uuid)
                            launch_without_gp = roslaunch.parent.ROSLaunchParent(uuid, [self.mpc_local_planner_launch_dir])
                            launch_without_gp.start()            
                            # Starting the Move Base Client
                            while not self.client.wait_for_server():
                                rospy.sleep(0.1)
                            # Send the goal pose to the move base client
                            goal = self.gcg.get_the_goal(self.pickup_point_list[0, :])
                            self.client.send_goal(goal)
                            self.time = rospy.Time.now()
                            # Wait for the goal to be reached
                            while self.status != 3 and self.status != 4 and not self.timeout:
                                self.status = self.client.get_state()
                                # print("Running Time: " + str(rospy.Time.now().to_sec() - self.time.to_sec()) + " Goal Status: " + str(self.status))
                                self.timeout = (rospy.Time.now().to_sec() - self.time.to_sec()) > self.move_base_client_timeout
                                # If new pose is received then write to the log file
                                if self.odom_stamp != self.previous_odom_stamp:
                                    # print(4)
                                    self.log_file_per_grid_point.write(str(rospy.Time.now()) + "," + str(self.current_pose_x) + "," + str(self.current_pose_y) + "," + str(self.current_pose_heading) + "," + str(self.current_twist_linear_x) + "," + str(self.current_twist_linear_y) + "," + str(self.current_twist_angular_x) + "," + str(self.current_twist_angular_y) + "," + str(self.current_twist_angular_z) + "," + str(self.current_vel_command) + "," + str(self.current_steering_command) + "," + str(self.status) + "\n")
                                    self.current_pose.append([self.current_pose_x, self.current_pose_y])
                                    self.previous_odom_stamp = self.odom_stamp
                        # Calculate the time elapsed
                        self.time_elapsed = rospy.Time.now().to_sec() - self.time.to_sec()
                        # Shutdown the mpc_local_planner
                        launch_without_gp.shutdown()
                        # Calculate the distance travelled
                        current_pose_array = np.array(self.current_pose)
                        self.distance_travelled = np.linalg.norm(current_pose_array[1:, :] - current_pose_array[:-1, :], axis=1).sum()
                        print("Distance Travelled: " + str(self.distance_travelled) + " Time Elapsed: " + str(self.time_elapsed) + " Status: " + str(self.status))
                        # Write to the whole log file
                        if self.status == 3:
                            self.goal_status = "SUCCEDDED"
                        elif self.status == 4:
                            self.goal_status = "ABORTED"
                        elif self.timeout:
                            self.goal_status = "TIMEOUT"
                        self.log_file_everything.write(str(x) + "," + str(y) + "," + str(yaw) + "," + str(self.distance_travelled) + "," + str(self.time_elapsed) + "," + str(self.goal_status) + "\n")
                        # Reset the variables
                        self.reset_variables()
                    
    def odom_callback(self, odom):
        self.odom_stamp = odom.header.stamp
        self.current_pose_x = odom.pose.pose.position.x
        self.current_pose_y = odom.pose.pose.position.y
        self.current_pose_z = odom.pose.pose.position.z
        current_pose_rpy = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
        self.current_pose_heading = current_pose_rpy[2]
        self.current_twist_linear_x = odom.twist.twist.linear.x
        self.current_twist_linear_y = odom.twist.twist.linear.y
        self.current_twist_angular_x = odom.twist.twist.angular.x
        self.current_twist_angular_y = odom.twist.twist.angular.y
        self.current_twist_angular_z = odom.twist.twist.angular.z
        pass
    
    def cmd_vel_callback(self, cmd_vel):
        self.current_vel_command = cmd_vel.linear.x
        self.current_steering_command = cmd_vel.angular.z
        pass
    
    def reset_variables(self):
        self.current_pose = []
        self.current_pose_x = None
        self.current_pose_y = None
        self.current_pose_heading = None
        self.current_twist_linear_x = None
        self.current_twist_linear_y = None
        self.current_twist_angular_x = None
        self.current_twist_angular_y = None
        self.current_twist_angular_z = None
        self.previous_pose_x = None
        self.previous_pose_y = None
        self.current_vel_command = None
        self.current_steering_command = None
        self.distance_travelled = 0
        self.time_elapsed = 0
        self.goal_status = None
        self.goal_reached = False
        self.status = None
        self.timeout = False
        pass
    
                    
if __name__ == '__main__':
    rospy.init_node('batch_sim', anonymous=True)
    batch_sim = BatchSim()
    while not rospy.is_shutdown():
        batch_sim.run_batch_sim()
        
