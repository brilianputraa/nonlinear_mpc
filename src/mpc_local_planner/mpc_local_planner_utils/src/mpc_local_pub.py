import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from mpc_local_planner_msgs import OptimalControlResult
from actionlib_msgs.msg import GoalStatus


pub = rospy.Publisher("/rbcar_robot_control/command", AckermannDriveStamped, queue_size=1)


def ctrl_callback(goal_stats):
    print(goal_stats)
    
    
def main():
    sub_goal = rospy.Subscriber("/move_base/status", GoalStatus, ctrl_callback, queue_size=1)
    # sub_ctrl = rospy.Subscriber("/move_base/MpcLocalPlannerROS/ocp_result", OptimalControlResult, queue_size=1)
    rospy.spin()
    

if __name__ == "__main__":
    rospy.init_node("mpc_local_pub_node", disable_signals=False)
    rate = rospy.Rate(20)
    try:
        main()
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
    