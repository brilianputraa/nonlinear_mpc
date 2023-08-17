#!/usr/bin/env bash

./newTabAndRun.sh 'cd
cd avp_ws/
source devel/setup.bash 
roslaunch mpc_local_planner_examples carlike_quadratic_form_with_bag.launch

sleep 2

./newTabAndRun.sh 
source devel/setup.bash 
roslaunch mpc_local_planner_utils data_logger_cart.launch'
