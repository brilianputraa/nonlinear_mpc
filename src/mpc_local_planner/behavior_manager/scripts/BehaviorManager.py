#!/usr/bin/env python2.7

import socket
import re
import struct

import rospy
import roslaunch
import numpy as np
import actionlib
import tf
import GoalCheckerandGetter as gcg
import GazeboHelper as gh

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from cgi import test


class BehaviorManager():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.PORT = rospy.get_param("~PORT_NUMBER")
        self.HOST = rospy.get_param("~SERVER_HOST")
        self.goal_lists_dir = rospy.get_param("~goal_lists_dir")
        self.pickup_point_list_dir = rospy.get_param("~pickup_point_list_dir")
        self.intemediate_goal_dir = rospy.get_param("~intermediate_goal_lists_dir")
        self.mpc_with_global_planner_dir = rospy.get_param("~mpc_with_global_planner_dir")
        self.mpc_without_global_planner_dir = rospy.get_param("~mpc_without_global_planner_dir")
        self.is_sim = rospy.get_param("~is_sim")
        # Using behavior manager without the server (for testing purposes)
        self.isTestWithoutServer = rospy.get_param("~isTestWithoutServer")
        ############## Testing purposes Only #################
        
        self.goal_point_list = np.loadtxt(self.goal_lists_dir, delimiter=',')
        self.pickup_point_list = np.loadtxt(self.pickup_point_list_dir, delimiter=',')
        self.intemediate_goal_dir = np.loadtxt(self.intemediate_goal_dir, delimiter=',')
        
        self.goal_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.move_base_status_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber("/INS/odom", Odometry, self.odom_cb, queue_size=1)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.mode_pub = rospy.Publisher("/driving_mode", String, queue_size=1)

        # Simulation Parameters (Gazebo)
        if self.is_sim:
            self.gazebo_helper = gh.GazeboHelper()
            self.starting_pose = [-19.8261, -12.2594, 0.097952, 0.0, 0.0, 0.0]
            self.starting_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                        
        self.parking_slot = None
        self.pickup_req = None
        self.scenario = None
        self.is_connected = False
        self.cart_parked = False
        self.waiting_time_before_parking = 20
        self.waiting_time_before_pickup = 20
        self.HEADER_SIZE = 10
        self.BUFFER_SIZE = 1024
        
        self.manual_mode = String()
        self.manual_mode.data = "m"
        self.autonomous_mode = String()
        self.autonomous_mode.data = "a"
        
        self.goal_utils = gcg.GoalCheckerandGetter()
        
        # Current Pose
        self.current_pose = [0.0, 0.0, 0.0]
        
        # Byte Data for the Server Communication
        self.parked_state_front = struct.pack('!B', 10)
        self.parked_state_middle = struct.pack('!B', 0)
        self.parked_state_back = struct.pack('!B', 0)
        self.parked_state = self.parked_state_front + self.parked_state_middle + self.parked_state_back
        
    def scenario_runner(self):        
        rospy.loginfo("Automated Valet Parking System is Running!")
        ############ Starting the Behavior Manager #############
        if not self.isTestWithoutServer:
            rospy.loginfo("Trying to Connect to the Server")
            
            while not self.is_connected:
            # Initializing the Socket Connection to the Server
                try:
                    self.sock.connect((self.HOST, self.PORT))
                    self.is_connected = True
                    print("Connection with Server Established!")
                except:
                    rospy.sleep(0.1)
            
            test_data = struct.pack('!B', 2) + struct.pack('!B', 0)
            self.sock.sendall(test_data)
            print("Data Sent!")
            
            # If in the simulation mode, reset the gazebo world
            if self.is_sim:
                rospy.loginfo("Resetting the Simulation World")
                self.gazebo_helper.setModelPoseAndSpeed("rbcar", self.starting_pose, self.starting_vel)
            
            # Waiting for the Server to Send the Scenario (Dropping off the passenger and go to the parking slot)
            while self.scenario != 1:
                rospy.loginfo("Waiting for the Server to Send the Scenario")
                bytes_data = self.sock.recv(self.BUFFER_SIZE)
                bytes_data = bytearray(bytes_data)
                self.scenario = bytes_data[0]
                self.parking_slot = int(bytes_data[1]) * 256 + bytes_data[2]
        else:
            self.scenario = 1
            self.parking_slot = 9

        print('Receive scenario: {:d}, slot number: {:d}'.format(self.scenario, self.parking_slot))

        # Starting the Move Base Server
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_with_gp = roslaunch.parent.ROSLaunchParent(uuid, [self.mpc_with_global_planner_dir])
        launch_with_gp.start()
         
        # Starting the Move Base Client and Checking the Server (1st phase)
        while not self.client.wait_for_server():
            rospy.sleep(0.1)
            
        rospy.loginfo("Move_base action server is on")
               
        # Activate the Autonomous Driving Mode
        self.mode_pub.publish(self.autonomous_mode)
        print("Received Data!")
        print("Parking Slot Number: ", self.parking_slot)

        # Start the Scenario (Getting the first goal): Dropping off the passenger and go to the parking slot
        if self.scenario == 1:
            rospy.loginfo("Dropping off the passenger")
            goal_points = self.goal_point_list[self.parking_slot - 1, 1:4]
            goal = self.goal_utils.get_the_goal(goal_points)
            print(goal)
            self.client.send_goal(goal)
            #TODO: Possibly not able to know the status of the goal (can be replaced with manually receiving the status from the server)
            self.client.wait_for_result()
            
            #TODO: Implement the roslaunch API to kill the current MPC and change the MPC parameters (2nd phase)
            if self.client.get_result():
                print("Goal is reached status: %s".format(self.client.get_result()))
                rospy.loginfo("Passenger successfully dropped off")
                launch_with_gp.shutdown()
                launch_without_gp = roslaunch.parent.ROSLaunchParent(uuid, [self.mpc_without_global_planner_dir])
                launch_without_gp.start()
            else:
                rospy.loginfo("Passenger drop off failed")
            
            # Starting the Move Base Client and Checking the Server (2nd phase)
            # self.client.wait_for_server()
            while not self.client.wait_for_server():
                rospy.sleep(0.1)
            rospy.loginfo("Move_base action server is on")
            
            for i in range(self.waiting_time_before_parking):
                print("Waiting before going to the parking slot: ", i)
            
            rospy.loginfo("Going to the parking slot")
            goal_points = self.goal_point_list[self.parking_slot - 1, 4:]
            goal = self.goal_utils.get_the_goal(goal_points)
            self.client.send_goal(goal)
            #TODO: Possibly not able to know the status of the goal (can be replaced with manually receiving the status from the server)
            self.client.wait_for_result()
            if self.client.get_result():
                print("Goal is reached status: %s".format(self.client.get_result()))
                rospy.loginfo("The golf-cart is parked")
                self.cart_parked = True
                # Change to the manual driving mode
                self.mode_pub.publish(self.manual_mode)
                if not self.isTestWithoutServer:
                    # Sending the parked state to the Server
                    self.sock.send(self.parked_state)
                print("Parked State Sent!")
                # Resetting the Scenario and to be ready for the next phase (Shutdown the local planner)
                launch_without_gp.shutdown()
            else:
                rospy.loginfo("Parking failed")
        # Waiting for the Server to Send the Scenario (When the server requests the pickup)
        if not self.isTestWithoutServer:
            rospy.loginfo("Waiting for the Server for Requesting Pickup")
            while True:
                bytes_data = self.sock.recv(self.BUFFER_SIZE)
                bytes_data = bytearray(bytes_data)
                self.scenario = bytes_data[0]
                self.pickup_req = int(bytes_data[1]) * 256 + bytes_data[2]
                rospy.loginfo("Received Data! Scenario: {}, Pickup Request: {}".format(self.scenario, self.pickup_req))
                
                ###################### Testing purposes (will be removed when the app is completed) #########################
                #self.scenario = 2
                #self.pickup_req = 0
                ############################################################################################################

                if self.scenario == 2 and self.pickup_req == 0:
                    rospy.loginfo("The Server has requested the pickup")
                    break
        else:
            rospy.sleep(5)
            self.scenario = 2
            self.pickup_req = 0
        
        # Start the Scenario (Getting the second goal): Picking up the passenger
        if self.scenario == 2 and self.cart_parked:
            launch_with_gp_pickup = roslaunch.parent.ROSLaunchParent(uuid, [self.mpc_without_global_planner_dir])
            for i in range(self.waiting_time_before_pickup):
                print("Waiting before picking up the passenger: ", i)
            # Activate the Autonomous Driving Mode
            self.mode_pub.publish(self.autonomous_mode)
            #TODO: Implement the roslaunch API to launch the MPC with the new parameters (3rd phase)
            launch_with_gp_pickup.start()
            # Starting the Move Base Client and Checking the Server (3rd phase)
            while not self.client.wait_for_server():
                rospy.sleep(0.1)
            rospy.loginfo("Move_base action server is on")  
            rospy.loginfo("Going to the passenger")
            # Intermediate goal points setter
            # First intermediate goal
            goal_points = self.goal_point_list[self.parking_slot - 1, 7:10]
            goal = self.goal_utils.get_the_goal(goal_points, pickup=False)
            self.client.send_goal(goal)
            #TODO: Possibly not able to know the status of the goal (can be replaced with manually receiving the status from the server)
            self.client.wait_for_result()
            if self.client.get_result():
                rospy.loginfo("1st Intermediate goal reached")
                # After the intermediate goal is reached, the goal is set to the actual pickup point
                goal_points = self.goal_point_list[self.parking_slot - 1, 10:]
                goal = self.goal_utils.get_the_goal(goal_points, pickup=False)
                self.client.send_goal(goal)
            else:
                rospy.loginfo("1st Intermediate goal failed")
            # Second Intermediate goal
            self.client.wait_for_result()
            if self.client.get_result():
                rospy.loginfo("2nd Intermediate goal reached")
                # After the intermediate goal is reached, the goal is set to the actual pickup point
                goal_points = self.pickup_point_list[0, :]
                goal = self.goal_utils.get_the_goal(goal_points, pickup=True)
                self.client.send_goal(goal)
            else:
                rospy.loginfo("2nd Intermediate goal failed")
            
            self.client.wait_for_result()
            # When the passenger is picked up successfully, reset the state of the scenario
            if self.client.get_result():
                rospy.loginfo("Passenger successfully picked up")
                self.cart_parked = False
                # Change to the manual driving mode
                self.mode_pub.publish(self.manual_mode)
                
                # Resetting the Scenario and the Parking Slot  
                self.is_connected = False
                self.scenario = None
                self.parking_slot = None
                self.cart_parked = False
                launch_with_gp_pickup.shutdown()
            else:
                rospy.loginfo("Passenger pickup failed")
            
    def move_base_status_cb(self, msg):
        if self.goal_utils.get_the_goal_reached_status(msg):
            pass
        
    def odom_cb(self, msg):
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y
        self.current_pose[2] = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        # rospy.loginfo("Current Pose: {}".format(self.current_pose))
        pass
            
class ActionStatus():
    def __init__(self):
        self.current_status = 0


if __name__ == '__main__':
    rospy.init_node("behavior_manager")
    behavior_manager = BehaviorManager()
    behavior_manager.scenario_runner()
    rospy.spin()
