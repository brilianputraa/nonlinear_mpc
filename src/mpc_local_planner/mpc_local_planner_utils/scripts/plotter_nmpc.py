import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


# File name
filename = "202308171756_mpc_local_planner.csv"

# Read data
logs = pd.read_csv(filename)

# Get data

# ROS Time
ros_time = logs.get(key="ros_time").to_numpy()
# Steering angle
steer = logs.get(key='mpc_steer').to_numpy()
# Velocity from NMPC
vel = logs.get(key="mpc_vel").to_numpy()
# Odometry velocity
odom_vel = logs.get(key="odom_vel").to_numpy()
# Odometry x axis
odom_x = logs.get(key="odom_x").to_numpy()
# Odometry y axis
odom_y = logs.get(key="odom_y").to_numpy()
# Odometry yaw axis
odom_yaw = logs.get(key="odom_yaw").to_numpy()
# Local planner path X-axis
get_path_x = []
temp_path_x = logs.get(key="path_x").to_numpy()

for x in range(len(temp_path_x)):
  path_x = temp_path_x[x][1:-2]
  path_x = [float(val) for val in path_x.split(',')]
  get_path_x.append(path_x[1])

# Local planner path y-axis
get_path_y = []
temp_path_y = logs.get(key="path_y").to_numpy()
for x in range(len(temp_path_y)):
  path_y = temp_path_y[x][1:-2]
  path_y = [float(val) for val in path_y.split(',')]
  get_path_y.append(path_y[1])

# Local planner path heading
get_path_yaw = []
temp_path_yaw = logs.get(key="path_yaw").to_numpy()
for x in range(len(temp_path_yaw)):
  path_yaw = temp_path_yaw[x][1:-2]
  path_yaw = [float(val) for val in path_yaw.split(',')]
  get_path_yaw.append(path_yaw[1])

# Steering Plot
plt.figure()
plt.plot(ros_time,steer)
plt.title("Steering Angle from NMPC Planner")
plt.xlabel("ROS Time")
plt.ylabel("Steering Angle (rad)")

# Velocity Plot
plt.figure()
plt.plot(ros_time,vel)
plt.plot(ros_time,odom_vel)
plt.legend(['Velocity from NMPC', 'Velocity Odom'])
plt.title("Velocity from NMPC Planner")
plt.xlabel("ROS Time")
plt.ylabel("Velocity (m/s)")

# Trajectory of the Vehicle
plt.figure()
plt.plot(odom_x, odom_y)
plt.plot(get_path_x, get_path_y)
plt.legend(['Vehicle Traj', 'Local Planner Path (t+1)'])
plt.title("Trajectory Results of Vehicle")
plt.xlabel("x (m)")
plt.ylabel("y (m)")

# Heading of the Vehicle
plt.figure()
plt.plot(ros_time,odom_yaw)
plt.plot(ros_time, get_path_yaw)
plt.legend(['Vehicle Heading', 'Local Planner Heading (t+1)'])
plt.title("Heading of Vehicle")
plt.xlabel("ROS Time")
plt.ylabel("Heading Angle (rad)")
