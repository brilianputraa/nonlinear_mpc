#include <ros/ros.h>

#include <Eigen/Core>

#include <mpc_local_planner/controller.h>
#include <mpc_local_planner/mpc_local_planner_ros.h>
#include <mpc_local_planner/utils/publisher.h>
#include <teb_local_planner/obstacles.h>

#include <memory>

namespace mpc = mpc_local_planner;

class MpcLocalOnly {
    public: 
        MpcLocalOnly() = default;

        void start(ros::NodeHandle& nh);

    protected:
        void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& msg);
        void CB_via_points(const nav_msgs::PathConstPtr& msg);
        void CB_odom(const nav_msgs::OdometryConstPtr& msg);
        void CB_custom_obstacles(const teb_local_planner::ObstaclesConstPtr& msg);

        teb_local_planner::ObstContainer _obstacles;
        std::vector<teb_local_planner::PoseSE2> _via_points;

};

void MpcLocalOnly::start(ros::NodeHandle& nh) {
    std::string map_frame = "map";

    // Clicked Goal Points Subscriber
    ros::Subscriber sub_clicked_points = nh.subscribe("move_base_simple/goal", 1, &MpcLocalOnly::CB_clicked_points, this);

    // Via Points Subscriber
    ros::Subscriber sub_via_points = nh.subscribe("/move_base/TebLocalPlannerROS/via_points", 1, &MpcLocalOnly::CB_via_points, this);

    // Robot Shape Model Getter
    teb_local_planner::RobotFootprintModelPtr robot_model = teb_local_planner::RobotFootprintModelPtr(new teb_local_planner::getRobotFootprintFromParamServer(nh));

    // Define Controller
    mpc::Controller controller;

    if (!controller.configure(nh)) {
        ROS_ERROR("Failed to configure controller");
        return;
    }

    // Define Publisher
    mpc::Publisher publisher(nh, controller.getRobotDynamics(), map_frame);

    // Odometry Subscriber
    ros::Subscriber sub_odom = nh.subscribe("/odom", 1, &MpcLocalOnly::CB_odom, this);

    teb_local_planner::PoseSE2 init_pose;
    teb_local_planner::PoseSE2 final_pose;

    corbo::TimeSeries::Ptr x_seq = std::make_shared<corbo::TimeSeries>();
    corbo::TimeSeries::Ptr u_seq = std::make_shared<corbo::TimeSeries>();

    geometry_msgs::Twist vel;
    bool success = false;

    ros::Rate rate(10);
    while (ros::ok()) {
        success = controller.step(init_pose, final_pose, _obstacles, vel, rate.expectedCycleTime().toSec(), ros::Time::now(), u_seq, x_seq);
        
        if (success) {
            publisher.publishLocalPlan(*x_seq);
        } else {
            ROS_WARN("Failed to find a solution of OCP");
        }

        publisher.publishRobotFootprintModel(init_pose, *robot_model);
        publisher.publishViaPoints(_via_points);
        ros::spinOnce();
        rate.sleep();
    }
}

void MpcLocalOnly::CB_clicked_points(const geometry_msgs::PoseStamped& msg) {
    _via_points.emplace_back(msg->point.x, msg->point.y, 0.0);
    ROS_INFO("Clicked Point: (%f, %f)", msg->point.x, msg->point.y);
}
