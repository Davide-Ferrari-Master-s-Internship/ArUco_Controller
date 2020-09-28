#include "useful_functions.h"


//------------------------------------------------ STANDARD FUNCTIONS -----------------------------------------------//


float max(float a,float b) {

    //return the max between two float

   if(a >= b) return a;
   else return b;

}


int Sign (float n) {

    if (n >= 0) return 1;
    else return -1;

}


float Degree_TO_Radiant (float deg) {

    //conversion degree to radiant
    return (deg * M_PI/180);

}


float Radiant_TO_Degree (float rad) {

    //conversion radiant to degree
    return (rad * 180/M_PI);

}


//------------------------------------------------ CMD_VEL FUNCTIONS ------------------------------------------------//


void cmd_vel_linear (geometry_msgs::Twist *cmd_vel, float x, float y, float z) {

    cmd_vel -> linear.x = x;
    cmd_vel -> linear.y = y;
    cmd_vel -> linear.z = z;

}


void cmd_vel_angular (geometry_msgs::Twist *cmd_vel, float x, float y, float z) {

    cmd_vel -> angular.x = x;
    cmd_vel -> angular.y = y;
    cmd_vel -> angular.z = z;
    
}


//-------------------------------------------------- ROS FUNCTIONS --------------------------------------------------//


void ROS_Sleep (float time) {

    ros::Duration t(time);
    ros::Duration(t).sleep();

}


//------------------------------------------------ MATRIX FUNCTIONS -------------------------------------------------//


void Matrix_Cout (std::string matrix_name, std::vector<std::vector<float>> matrix) {
    

    std::cout << std::endl << matrix_name << ":" << std::endl;

    for (int i = 0; i < matrix.size(); i++) {           //rows

        for (int j = 0; j < matrix[i].size(); j++) {   //columns

            std::cout << matrix[i][j] << "   ";
        
        }

        std::cout << std::endl;

    }

}


void Rotation_Matrix_Cout (std::string matrix_name, Eigen::Matrix3d rotation_matrix) {

    std::cout << std::endl << matrix_name << ":" << std::endl;

    for (int i = 0; i < 3; i++) {       //rows

        for (int j = 0; j < 3; j++) {   //columns

            std::cout << rotation_matrix(i,j) << "   ";
        
        }

        std::cout << std::endl;

    }

}


void Transformation_Matrix_Cout (std::string matrix_name, Eigen::Matrix4d transformation_matrix) {

    std::cout << std::endl << matrix_name << ":" << std::endl;

    for (int i = 0; i < 4; i++) {       //rows

        for (int j = 0; j < 4; j++) {   //columns

            std::cout << transformation_matrix(i,j) << "   ";
        
        }

        std::cout << std::endl;

    }

}


Eigen::Matrix3d Rotation_Matrix (float r0c0, float r0c1, float r0c2, float r1c0, float r1c1, float r1c2, float r2c0, float r2c1, float r2c2) {

    Eigen::Matrix3d rotation_matrix;

    rotation_matrix(0,0) = r0c0;
    rotation_matrix(0,1) = r0c1;
    rotation_matrix(0,2) = r0c2;
    rotation_matrix(1,0) = r1c0;
    rotation_matrix(1,1) = r1c1;
    rotation_matrix(1,2) = r1c2;
    rotation_matrix(2,0) = r2c0;
    rotation_matrix(2,1) = r2c1;
    rotation_matrix(2,2) = r2c2;

    return rotation_matrix;

}


Eigen::Matrix3d Rotation_Matrix (std::vector<double> rotation_quaternion) {

    Eigen::Quaterniond quaternion;

    quaternion.x() = rotation_quaternion[0];
    quaternion.y() = rotation_quaternion[1];
    quaternion.z() = rotation_quaternion[2];
    quaternion.w() = rotation_quaternion[3];

    Eigen::Matrix3d rotation_matrix(quaternion);

    return rotation_matrix;

}


Eigen::Matrix3d Rotation_Matrix (geometry_msgs::Pose pose) {

    Eigen::Quaterniond quaternion;

    quaternion.x() = pose.orientation.x;
    quaternion.y() = pose.orientation.y;
    quaternion.z() = pose.orientation.z;
    quaternion.w() = pose.orientation.w;

    Eigen::Matrix3d rotation_matrix(quaternion);

    return rotation_matrix;

}


Eigen::Matrix4d Transformation_Matrix (std::vector<double> translation_vector, std::vector<double> rotation_vector) {

    Eigen::Quaterniond quaternion;

    quaternion.x() = rotation_vector[0];
    quaternion.y() = rotation_vector[1];
    quaternion.z() = rotation_vector[2];
    quaternion.w() = rotation_vector[3];

    //Rotation Matrix and Translation Vector
    Eigen::Matrix3d rotation_matrix(quaternion);
    Eigen::Vector3d translation_vec;

    translation_vec.x() = translation_vector[0];
    translation_vec.y() = translation_vector[1];
    translation_vec.z() = translation_vector[2];

    //Transformation Matrix
    Eigen::Matrix4d transformation;

    //Set Identity to make bottom row of Matrix 0,0,0,1
    transformation.setIdentity();

    transformation.block<3,3>(0,0) = rotation_matrix;
    transformation.block<3,1>(0,3) = translation_vec;

    return transformation;

}


Eigen::Matrix4d Transformation_Matrix (geometry_msgs::Pose pose) {

    Eigen::Quaterniond quaternion;

    quaternion.x() = pose.orientation.x;
    quaternion.y() = pose.orientation.y;
    quaternion.z() = pose.orientation.z;
    quaternion.w() = pose.orientation.w;

    //Rotation Matrix and Translation Vector
    Eigen::Matrix3d rotation_matrix(quaternion);
    Eigen::Vector3d translation_vec;

    translation_vec.x() = pose.position.x;
    translation_vec.y() = pose.position.y;
    translation_vec.z() = pose.position.z;

    //Transformation Matrix
    Eigen::Matrix4d transformation;

    //Set Identity to make bottom row of Matrix 0,0,0,1
    transformation.setIdentity();

    transformation.block<3,3>(0,0) = rotation_matrix;
    transformation.block<3,1>(0,3) = translation_vec;

    return transformation;

}


std::vector<double> Get_Translation (Eigen::Matrix4d transformation_matrix) {

    Eigen::Vector3d translation_vector = transformation_matrix.block<3,1>(0,3);
    std::vector<double> translation;
    translation.resize(3);

    translation[0] = translation_vector.x();
    translation[1] = translation_vector.y();
    translation[2] = translation_vector.z();

    return translation;

}


std::vector<double> Get_Rotation_Quaternion (Eigen::Matrix4d transformation_matrix) {

    Eigen::Matrix3d rotation_matrix = transformation_matrix.block<3,3>(0,0);
    Eigen::Quaterniond quaternion(rotation_matrix);
    std::vector<double> rotation;
    rotation.resize(4);

    rotation[0] = quaternion.x();
    rotation[1] = quaternion.y();
    rotation[2] = quaternion.z();
    rotation[3] = quaternion.w();

    return rotation;

}


geometry_msgs::Pose Get_Pose_From_Transformation (Eigen::Matrix4d transformation_matrix) {

    Eigen::Matrix3d rotation_matrix = transformation_matrix.block<3,3>(0,0);
    Eigen::Quaterniond quaternion(rotation_matrix);
    Eigen::Vector3d translation_vector = transformation_matrix.block<3,1>(0,3);

    geometry_msgs::Pose pose;

    pose.position.x = translation_vector.x();
    pose.position.y = translation_vector.y();
    pose.position.z = translation_vector.z();

    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    return pose;
}


std::vector<double> Quaternion_TO_Vector (Eigen::Quaterniond quaternion) {

    std::vector<double> rotation;
    rotation.resize(4);

    rotation[0] = quaternion.x();
    rotation[1] = quaternion.y();
    rotation[2] = quaternion.z();
    rotation[3] = quaternion.w(); 

    return rotation;

}


Eigen::Quaterniond Quaternion_Identity (void) {

    //quaternion_identity = {0,0,0,1};
    Eigen::Quaterniond quaternion_identity;

    quaternion_identity.x() = 0;
    quaternion_identity.y() = 0;
    quaternion_identity.z() = 0;
    quaternion_identity.w() = 1;

    return quaternion_identity;

}
