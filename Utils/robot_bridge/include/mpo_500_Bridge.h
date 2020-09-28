#ifndef MPO_500_BRIDGE_H
#define MPO_500_BRIDGE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "robot_bridge/planned_trajectory_msg.h"
#include "robot_bridge/geometry_trajectory_msg.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <math.h>


class mpo_500_bridge {

    public:

        mpo_500_bridge();

        void spinner (void);

    private:

        ros::NodeHandle nh;

        ros::Publisher cmd_vel_publisher;

        ros::Subscriber velocity_subscriber;
        ros::Subscriber trajectory_subscriber;

        geometry_msgs::Twist cmd_vel, IO_cmd_vel;
        robot_bridge::geometry_trajectory_msg trajectory;
        
        void Planned_Velocity_Callback (const geometry_msgs::TwistConstPtr &);
        void Planned_Trajectory_Callback (const robot_bridge::geometry_trajectory_msg::ConstPtr &);

        void Plan_IO_SFL (robot_bridge::geometry_trajectory_msg trajectory);

        void Publish_Velocity (geometry_msgs::Twist velocity);
        
        

};

#endif /* MPO_500_BRIDGE_H */