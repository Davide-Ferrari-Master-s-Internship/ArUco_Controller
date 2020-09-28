#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "Eigen/Dense"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <math.h>
#include <cmath>

#define _USE_MATH_DEFINES

//------------------------------------------------ STANDARD FUNCTIONS -----------------------------------------------//

float max(float a,float b);
int Sign (float n);
float Degree_TO_Radiant (float deg);
float Radiant_TO_Degree (float rad);

//------------------------------------------------ CMD_VEL FUNCTIONS ------------------------------------------------//

void cmd_vel_linear (geometry_msgs::Twist *cmd_vel, float x, float y, float z);
void cmd_vel_angular (geometry_msgs::Twist *cmd_vel, float x, float y, float z);

//-------------------------------------------------- ROS FUNCTIONS --------------------------------------------------//

void ROS_Sleep (float time);

//------------------------------------------------ MATRIX FUNCTIONS -------------------------------------------------//

void Matrix_Cout (std::string matrix_name, std::vector<std::vector<float>> matrix);
void Rotation_Matrix_Cout (std::string matrix_name, Eigen::Matrix3d rotation_matrix);
void Transformation_Matrix_Cout (std::string matrix_name, Eigen::Matrix4d transformation_matrix);
Eigen::Matrix3d Rotation_Matrix (float r0c0, float r0c1, float r0c2, float r1c0, float r1c1, float r1c2, float r2c0, float r2c1, float r2c2);
Eigen::Matrix3d Rotation_Matrix (std::vector<double> rotation_quaternion);
Eigen::Matrix3d Rotation_Matrix (geometry_msgs::Pose pose);
Eigen::Matrix4d Transformation_Matrix (std::vector<double> translation_vector, std::vector<double> rotation_vector);
Eigen::Matrix4d Transformation_Matrix (geometry_msgs::Pose pose);
std::vector<double> Get_Translation (Eigen::Matrix4d transformation_matrix);
std::vector<double> Get_Rotation_Quaternion (Eigen::Matrix4d transformation_matrix);
geometry_msgs::Pose Get_Pose_From_Transformation (Eigen::Matrix4d transformation_matrix);
std::vector<double> Quaternion_TO_Vector (Eigen::Quaterniond quaternion);
Eigen::Quaterniond Quaternion_Identity (void);

