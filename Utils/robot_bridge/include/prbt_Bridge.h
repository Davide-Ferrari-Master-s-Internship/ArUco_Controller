#ifndef PRBT_BRIDGE_H
#define PRBT_BRIDGE_H

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "robot_bridge/planned_trajectory_msg.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <math.h>
#include <vector>


class prbt_bridge {

    public:

        prbt_bridge();
        
        void spinner (void); 

    private:

        ros::NodeHandle nh;

        ros::Publisher manipulator_joint_trajectory_controller_command_publisher;
        ros::Publisher trajectory_counter_publisher;
        ros::Publisher current_state_position_publisher, prbt_position_reached_publisher;

        ros::Subscriber trajectory_subscriber;
        ros::Subscriber current_position_subscriber;

        trajectory_msgs::JointTrajectory planned_trajectory, next_point;
        control_msgs::JointTrajectoryControllerState current_position;

        std_msgs::Bool position_reached;

        ros::Time begin;

        int trajectory_counter = 0;
        bool idle_publisher = true;
        float tolerance = 0;
        float sampling_time = 0;
        float position_error = 0;

        void Planned_Trajectory_Callback (const trajectory_msgs::JointTrajectory::ConstPtr &);
        void Current_Position_Callback (const control_msgs::JointTrajectoryControllerState::ConstPtr &);

        void Compute_Tolerance(trajectory_msgs::JointTrajectory planned_trajectory);
        float Compute_Position_Error (void);
        void Wait_For_Desired_Position (void);

        void Check_Joint_Limits (trajectory_msgs::JointTrajectory *point);
        void Next_Goal (trajectory_msgs::JointTrajectory planned_trajectory, int counter);

        // trajectory_msgs::JointTrajectory Current_Position_To_Next_Goal (control_msgs::JointTrajectoryControllerState position);
        // void Current_Position_Maintainment (void);

        void Publish_Next_Goal (trajectory_msgs::JointTrajectory goal);
        void Publish_Trajectory_Counter (int counter);
        void Publish_Current_Position (control_msgs::JointTrajectoryControllerState position);
        
        
};

#endif /* PRBT_BRIDGE_H */