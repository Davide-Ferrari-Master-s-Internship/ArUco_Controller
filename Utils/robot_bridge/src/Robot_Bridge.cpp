#include "Robot_Bridge.h"


bridge::bridge() {

    //Initialize subscriber and Publisher
    rossini_trajectory_subscriber = nh.subscribe("/Robot_Bridge/Rossini_Planned_Trajectory", 1000, &bridge::Rossini_Planned_Trajectory_Callback, this);

    prbt_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/Robot_Bridge/prbt_Planned_Trajectory", 1000);

    mpo_500_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/Robot_Bridge/mpo_500_Planned_Velocity", 1000);
    mpo_500_trajectory_publisher = nh.advertise<robot_bridge::geometry_trajectory_msg>("/Robot_Bridge/mpo_500_Planned_Trajectory", 1000);


}


void bridge::Rossini_Planned_Trajectory_Callback(const robot_bridge::planned_trajectory_msg::ConstPtr &planned_trajectory_msg) {

    rossini_trajectory = *planned_trajectory_msg;

    prbt_trajectory = rossini_trajectory.manipulator_trajectory;

    mpo_500_velocity = rossini_trajectory.base_velocity;
    mpo_500_trajectory = rossini_trajectory.base_trajectory;

    prbt_trajectory_publisher.publish(prbt_trajectory);
    mpo_500_velocity_publisher.publish(mpo_500_velocity);
    mpo_500_trajectory_publisher.publish(mpo_500_trajectory);

}


void bridge::spinner (void) {

    ros::spinOnce();

}
