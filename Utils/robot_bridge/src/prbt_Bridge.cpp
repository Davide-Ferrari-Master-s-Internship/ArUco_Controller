#include "prbt_Bridge.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//


prbt_bridge::prbt_bridge () {

    //Initialize publishers and subscribers"

    trajectory_subscriber = nh.subscribe("/Robot_Bridge/prbt_Planned_Trajectory", 1000, &prbt_bridge::Planned_Trajectory_Callback, this);
    current_position_subscriber = nh.subscribe("/prbt/manipulator_joint_trajectory_controller/state", 1000, &prbt_bridge::Current_Position_Callback, this);

    manipulator_joint_trajectory_controller_command_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/prbt/manipulator_joint_trajectory_controller/command", 1000);
    trajectory_counter_publisher = nh.advertise<std_msgs::Int32>("/Robot_Bridge/prbt_Trajectory_Counter", 1);
    current_state_position_publisher = nh.advertise<control_msgs::JointTrajectoryControllerState>("/Robot_Bridge/prbt_Current_State_Position", 1000);
    prbt_position_reached_publisher = nh.advertise<std_msgs::Bool>("/Robot_Bridge/prbt_Position_Reached", 1);

    current_position.joint_names = {"empty", "empty", "empty", "empty", "empty", "empty"};

}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void prbt_bridge::Planned_Trajectory_Callback (const trajectory_msgs::JointTrajectory::ConstPtr &planned_trajectory_msg) {

    planned_trajectory = *planned_trajectory_msg;

}

void prbt_bridge::Current_Position_Callback (const control_msgs::JointTrajectoryControllerState::ConstPtr &current_position_msg) {

    current_position = *current_position_msg;
    Publish_Current_Position(current_position);

}


//----------------------------------------------------- PUBLISHER ------------------------------------------------------//


void prbt_bridge::Publish_Next_Goal (trajectory_msgs::JointTrajectory goal) {

    ROS_INFO("PRBT GOTO (%.2f;%.2f;%.2f;%.2f;%.2f;%.2f)", goal.points[0].positions[0], goal.points[0].positions[1], goal.points[0].positions[2], goal.points[0].positions[3], goal.points[0].positions[4], goal.points[0].positions[5]);

    manipulator_joint_trajectory_controller_command_publisher.publish(goal);

}

void prbt_bridge::Publish_Trajectory_Counter (int counter) {

    ROS_INFO("Trajectory Counter: %d", counter);

    std_msgs::Int32 count;

    count.data = counter;

    trajectory_counter_publisher.publish(count);

}

void prbt_bridge::Publish_Current_Position (control_msgs::JointTrajectoryControllerState position) {

    current_state_position_publisher.publish(position);

}


//----------------------------------------------------- FUNCTIONS -----------------------------------------------------//

void prbt_bridge::Check_Joint_Limits (trajectory_msgs::JointTrajectory *point) {

    float joint_limits[6] = {2.967, 2.095, 2.356, 2.967, 2.966, 3.123};

    for (int i = 0; i < 6; i++) {

        if (point->points[0].positions[i] > joint_limits[i]) {
            
            point->points[0].positions[i] = joint_limits[i];

            ROS_WARN("Joint_%d Limit Exceeded [MAX = %.3f]", i+1, joint_limits[i]);
            
        } else if (point->points[0].positions[i] < -joint_limits[i]) {

            point->points[0].positions[i] = -joint_limits[i];

            ROS_WARN("Joint_%d Limit Exceeded [MIN = %.3f]", i+1, -joint_limits[i]);

        }

    }

}

void prbt_bridge::Next_Goal (trajectory_msgs::JointTrajectory planned_trajectory, int counter/*, float sampling_time*/) {

    next_point.joint_names = planned_trajectory.joint_names;
    next_point.points.resize(1);
    next_point.points[0] = planned_trajectory.points[counter];

    if (next_point.points[0].time_from_start.toSec() == 0) {
        
        next_point.points[0].time_from_start = ros::Duration (sampling_time/1000);
        
    } else {next_point.points[0].time_from_start = ros::Duration (sampling_time);}

    Check_Joint_Limits(&next_point);

}

/* trajectory_msgs::JointTrajectory prbt_bridge::Current_Position_To_Next_Goal (control_msgs::JointTrajectoryControllerState position) {

    trajectory_msgs::JointTrajectory goal;

    goal.joint_names = position.joint_names;
    goal.points.resize(1);
    goal.points[0] = position.actual;

    goal.points[0].time_from_start = ros::Duration (0.1);

    return goal;
} */

/* void prbt_bridge::Current_Position_Maintainment (void) {

    if (idle_publisher) {
        
        begin = ros::Time::now();
        idle_publisher = false;
    
    }

    //wait 3 seconds
    if ((current_position.joint_names[0] != "empty") && ((ros::Time::now() - begin).toSec() >= 30) && !idle_publisher) {

        ros::spinOnce();
        
        Publish_Next_Goal(Current_Position_To_Next_Goal(current_position));

        current_position.joint_names = {"empty", "empty", "empty", "empty", "empty", "empty"};
        idle_publisher = true;

    }
} */

void prbt_bridge::Compute_Tolerance (trajectory_msgs::JointTrajectory planned_trajectory) {

    if (planned_trajectory.points[0].time_from_start.toSec() == 0) {

        sampling_time = fabs((planned_trajectory.points[1].time_from_start).toSec());

    } else {sampling_time = fabs((planned_trajectory.points[0].time_from_start).toSec());}
    

    /*

    5° Equation (5 sperimental points):     y = a*x^4 + b*x^3 + c*x^2 + d*x

    Points (sampling time, tolerance):

    A(0.01, 0.0001)     B(0.05, 0.005)     C(0.1, 0.02)     D(0.5, 0.1)
    
    */


    float a = -3.4013605442, b = -2.2335600907, c = 2.3945578231, d = -0.0137188209;

    tolerance = fabs(a*pow(sampling_time,4) + a*pow(sampling_time,3) + c*pow(sampling_time,2) + d*(sampling_time));

    ROS_INFO("Sampling Time: %f",sampling_time);
    ROS_INFO("Position Tolerance: %f",tolerance);

}

float prbt_bridge::Compute_Position_Error (void) {

    std::vector<float> error {0,0,0,0,0,0};
    for (int i = 0; i < 6; i++) {error[i] = fabs(current_position.desired.positions[i] - next_point.points[0].positions[i]);}

    return (*std::max_element(error.begin(), error.end()));

}

void prbt_bridge::Wait_For_Desired_Position (void) {

    position_error = tolerance + 1;         //needed to enter the while condition  

    while ((position_error > tolerance) && (fabs((begin - ros::Time::now()).toSec()) < (sampling_time * 0.85))) {    //wait for reaching desired position

        position_error = Compute_Position_Error();

        ros::spinOnce();

    }

}


//------------------------------------------------------- MAIN --------------------------------------------------------//


void prbt_bridge::spinner (void) {

    ros::spinOnce();

    while ((trajectory_counter < planned_trajectory.points.size()) && (planned_trajectory.points.size() != 0)) {

        if (trajectory_counter == 0) {  //new trajectory

            Compute_Tolerance(planned_trajectory);

        }

        //final position not reached
        position_reached.data = false;
        prbt_position_reached_publisher.publish(position_reached);

        //check the trajectory for dynamic replanning
        ros::spinOnce();

        Next_Goal(planned_trajectory, trajectory_counter);  //compute next goal

        Publish_Trajectory_Counter(trajectory_counter);     //publish on topic "PC_Controller/prbt_Trajectory_Counter"
        Publish_Next_Goal(next_point);                      //publish next goal on prbt topic
        
        begin = ros::Time::now();
        Wait_For_Desired_Position();    //wait until the tolerance is achieved

        trajectory_counter++;

    }

    if (trajectory_counter != 0) {

            //final position reached
            position_reached.data = true;
            prbt_position_reached_publisher.publish(position_reached);

            trajectory_counter = 0;

            planned_trajectory.points.clear();
            next_point.points.clear();

            idle_publisher = true;

    }


    // Current_Position_Maintainment();     //in order to avoit robot crash -> suspended


}