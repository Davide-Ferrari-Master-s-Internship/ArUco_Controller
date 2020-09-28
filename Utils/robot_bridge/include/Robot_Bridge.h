#ifndef ROBOT_BRIDGE_H
#define ROBOT_BRIDGE_H

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Twist.h"
#include "robot_bridge/planned_trajectory_msg.h"
#include "robot_bridge/geometry_trajectory_msg.h"


class bridge {

    public:

        bridge();

        void spinner();
        
    private:

        ros::NodeHandle nh;
        
        ros::Publisher prbt_trajectory_publisher, mpo_500_velocity_publisher, mpo_500_trajectory_publisher;
        ros::Subscriber rossini_trajectory_subscriber;

        robot_bridge::planned_trajectory_msg rossini_trajectory;

        trajectory_msgs::JointTrajectory prbt_trajectory;        
        geometry_msgs::Twist mpo_500_velocity;
        robot_bridge::geometry_trajectory_msg mpo_500_trajectory;
        
        void Rossini_Planned_Trajectory_Callback(const robot_bridge::planned_trajectory_msg::ConstPtr &);


};

#endif /* ROBOT_BRIDGE_H */