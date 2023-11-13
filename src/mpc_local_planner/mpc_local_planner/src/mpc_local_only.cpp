#include <ros/ros.h>
#include <tf/tf.h>

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
        teb_local_planner::PoseSE2 init_pose;
        teb_local_planner::PoseSE2 final_pose;
        geometry_msgs::Twist robot_vel;
        double dt = 0.1;

        // Costmap
        _costmap_ros = costmap_ros;
        _costmap     = _costmap_ros->getCostmap(); 

        _costmap_model = std::make_shared<base_local_planner::CostmapModel>(*_costmap);

        // Robot Pose
        geometry_msgs::PoseStamped robot_pose;
        _costmap_ros->getRobotPose(robot_pose);
        _robot_pose = PoseSE2(robot_pose.pose);

        // reserve some memory for obstacles
        _obstacles.reserve(700);

    protected:
        void CB_clicked_points(const geometry_msgs::PoseStampedConstPtr& msg);
        void CB_odom(const nav_msgs::OdometryConstPtr& msg);
        void updateObstacleContainerWithCostmap();

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
    ros::Subscriber sub_odom = nh.subscribe("/odoms", 1, &MpcLocalOnly::CB_odom, this);

    corbo::TimeSeries::Ptr x_seq = std::make_shared<corbo::TimeSeries>();
    corbo::TimeSeries::Ptr u_seq = std::make_shared<corbo::TimeSeries>();

    bool success = false;

    ros::Rate rate(10);
    while (ros::ok()) {
        success = controller.step(init_pose, final_pose, _obstacles, robot_vel, dt, ros::Time::now(), u_seq, x_seq);
        
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

void MpcLocalOnly::CB_clicked_points(const geometry_msgs::PoseStampedConstPtr& msg) {
    final_pose = teb_local_planner::PoseSE2(msg->pose.position.x, msg->pose.position.y, tf::getYaw(msg->pose.orientation));
    _via_points.emplace_back(msg->pose.position.x, msg->pose.position.y, tf::getYaw(msg->pose.orientation));
    ROS_INFO("Clicked Point: (%f, %f)", msg->point.x, msg->point.y);
}

void MpcLocalOnly::CB_odom(const nav_msgs::OdometryConstPtr& msg) {
    robot_vel = msg->twist.twist;
    init_pose = teb_local_planner::PoseSE2(msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));
}

void MpcLocalOnly::updateObstacleContainerWithCostmap()
{
    // Add costmap obstacles if desired
    Eigen::Vector2d robot_orient = _robot_pose.orientationUnitVec();

    for (unsigned int i = 0; i < _costmap->getSizeInCellsX() - 1; ++i)
    {
        for (unsigned int j = 0; j < _costmap->getSizeInCellsY() - 1; ++j)
        {
            if (_costmap->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
            {
                Eigen::Vector2d obs;
                _costmap->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

                // check if obstacle is interesting (e.g. not far behind the robot)
                Eigen::Vector2d obs_dir = obs - _robot_pose.position();
                if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > 2.0) continue;

                _obstacles.push_back(ObstaclePtr(new PointObstacle(obs)));
            }
        }
    }
}
