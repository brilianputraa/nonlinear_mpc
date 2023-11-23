#!/usr/bin/env python2.7

import rospy

from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock


pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
time_now = None
publish_once = True


def clock_cb(clock):
    global publish_once
    
    publish_once = False
    goal_point = PoseStamped()
    
    print("Time ", rospy.Time.now())
    goal_point.header.frame_id = 'world'
    goal_point.header.stamp = rospy.Time.now()
    goal_point.header.seq = 1
    
    goal_point.pose.position.x = 23.6302177750589
    goal_point.pose.position.y = 0.392362376175472
    goal_point.pose.position.z = 0.0
    
    goal_point.pose.orientation.x = 0.0
    goal_point.pose.orientation.y = 0.0
    goal_point.pose.orientation.z = 0.0
    goal_point.pose.orientation.w = 1.0
    
    print(goal_point)
    
    pub_goal.publish(goal_point)    
        


def main():
    sub_clock = rospy.Subscriber("/clock", Clock, clock_cb, queue_size=1)
    if time_now is None:
        print("Time is None")
    rospy.spin()
    
    
if __name__ == "__main__":
    rospy.init_node("goal_node", disable_signals=False)
    rate = rospy.Rate(10)
    try:
        main()
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
