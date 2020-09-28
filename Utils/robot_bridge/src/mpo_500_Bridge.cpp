#include "mpo_500_Bridge.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//


mpo_500_bridge::mpo_500_bridge() {

    //Initialize publishers and subscribers"

    cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    velocity_subscriber = nh.subscribe("/Robot_Bridge/mpo_500_Planned_Velocity", 1000, &mpo_500_bridge::Planned_Velocity_Callback, this);
    trajectory_subscriber = nh.subscribe("/Robot_Bridge/mpo_500_Planned_Trajectory", 1000, &mpo_500_bridge::Planned_Trajectory_Callback, this);

}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void mpo_500_bridge::Planned_Velocity_Callback (const geometry_msgs::Twist::ConstPtr &cmd_vel_msg) {

    cmd_vel = *cmd_vel_msg;

    Publish_Velocity(cmd_vel);

}

void mpo_500_bridge::Planned_Trajectory_Callback (const robot_bridge::geometry_trajectory_msg::ConstPtr &trajectory_msg) {

    trajectory = *trajectory_msg;
    
    for (int i = 0; i < trajectory.base_path.size(); i++) {

        ROS_WARN("Trajectory[%d]: %f , %f",i,trajectory.base_path[i].x,trajectory.base_path[i].y);

    }

}


//----------------------------------------------------- PUBLISHER ------------------------------------------------------//


void mpo_500_bridge::Publish_Velocity (geometry_msgs::Twist velocity) {

    cmd_vel_publisher.publish(velocity);

}


//----------------------------------------------------- FUNCTIONS -----------------------------------------------------//


void mpo_500_bridge::Plan_IO_SFL (robot_bridge::geometry_trajectory_msg trajectory) {
    
    float v_dx, v_dy, x_b, y_b, x_des, y_des, vx_des, vy_des, v, w;
    float pi = 3.1415, k = 1, b = 0.1;
    float x = 0, y = 0, theta = 0;

    for (int i = 0; i < trajectory.base_path.size(); i++) {

        x_b = x + b * cos(theta);
        y_b = y + b * sin(theta);

        x_des = trajectory.base_path[i].x;
        y_des = trajectory.base_path[i].y;

        vx_des = (x_des - x) / trajectory.time_from_start[i].toSec();     //x-derivative
        vy_des = (y_des - y) / trajectory.time_from_start[i].toSec();     //y-derivative

        v_dx = vx_des + k * (x_des - x_b);
        v_dy = vy_des + k * (y_des - y_b);

        v = cos(theta) * v_dx + sin(theta) * v_dy;
        w = (-1 / b) * sin(theta) * v_dx + (1/b) * cos(theta) * v_dy;

        IO_cmd_vel.linear.x = v;
        IO_cmd_vel.angular.z = w;

        ROS_INFO("x = %f, y = %f, theta = %f,\n xb = %f, yb = %f, xdes = %f, ydes = %f", x,y,theta,x_b,y_b,x_des,y_des);
        ROS_INFO("vx_des = %f, vy_des = %f, \n vdx = %f, vdy = %f, time = %f", vx_des,vy_des,v_dx,v_dy,trajectory.time_from_start[i].toSec());
        ROS_INFO("v = %f, w = %f", v,w);

        Publish_Velocity(IO_cmd_vel);     //controlled in position (x,y)

        ros::Duration(trajectory.time_from_start[i].toSec() - trajectory.time_from_start[i].toSec()/100).sleep();

        x = x + v * cos(theta) * trajectory.time_from_start[i].toSec();
        y = y + v * sin(theta) * trajectory.time_from_start[i].toSec();
        theta = theta + w * trajectory.time_from_start[i].toSec();

    }

}


//------------------------------------------------------- MAIN --------------------------------------------------------//


void mpo_500_bridge::spinner (void) {

    ros::spinOnce();

    //Velocity Callback -> Republish Velocity on Topic: /cmd_vel

    /*

    if (trajectory.base_path.size() != 0) {
        
        Plan_IO_SFL(trajectory);          //IO_SFL (x,y -> v,w)

        trajectory.base_path.clear();
        trajectory.time_from_start.clear();

    }

    */

}
