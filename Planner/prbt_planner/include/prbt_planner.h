#ifndef PRBT_PLANNER_H
#define PRBT_PLANNER_H

#include "ros/ros.h"

#include "prbt_planner/PlanJointSpace.h"
#include "prbt_planner/PlanCartesianSpace.h"
#include "prbt_planner/SetPlannerParameters.h"
#include "prbt_planner/SetPlannerDynamics.h"
#include "prbt_planner/ComputeEndEffectorPosition.h"
#include "prbt_planner/GetManipulatorInfo.h"
#include "prbt_planner/GetManipulatorLinkNames.h"
#include "prbt_planner/GetManipulatorTrajectory.h"

#include "dynamic_planner/dynamic_planner.h"
#include "trajectory_msgs/JointTrajectory.h"

class prbt_planner_class {

    public:

        prbt_planner_class();
        ~prbt_planner_class();

        void spinner (void);

        bool Plan_Joint_Space_Callback(prbt_planner::PlanJointSpace::Request &req, prbt_planner::PlanJointSpace::Response &res);
        bool Plan_Cartesian_Space_Callback(prbt_planner::PlanCartesianSpace::Request &req, prbt_planner::PlanCartesianSpace::Response &res);
        
        bool Set_Planner_Parameters_Callback(prbt_planner::SetPlannerParameters::Request &req, prbt_planner::SetPlannerParameters::Response &res);
        bool Set_Planner_Dynamics_Callback(prbt_planner::SetPlannerDynamics::Request &req, prbt_planner::SetPlannerDynamics::Response &res);
        
        bool Compute_End_Effector_Position_Callback(prbt_planner::ComputeEndEffectorPosition::Request &req, prbt_planner::ComputeEndEffectorPosition::Response &res);

        bool Get_Manipulator_Info_Callback(prbt_planner::GetManipulatorInfo::Request &req, prbt_planner::GetManipulatorInfo::Response &res);
        bool Get_Manipulator_Link_Names_Callback(prbt_planner::GetManipulatorLinkNames::Request &req, prbt_planner::GetManipulatorLinkNames::Response &res);
        bool Get_Manipulator_Trajectory_Callback(prbt_planner::GetManipulatorTrajectory::Request &req, prbt_planner::GetManipulatorTrajectory::Response &res);

    private:

        ros::NodeHandle nh;
    
        ros::ServiceServer plan_joint_space_server;
        ros::ServiceServer plan_cartesian_space_server;
        ros::ServiceServer set_planner_parameters_server;
        ros::ServiceServer set_planner_dynamics_server;
        ros::ServiceServer compute_end_effector_position_server;
        ros::ServiceServer get_manipulator_info_server;
        ros::ServiceServer get_manipulator_link_names_server;
        ros::ServiceServer get_manipulator_trajectory_server;
        
        struct manipulator_link_names {
            std::string world;
            std::string base;
            std::vector<std::string> links;
            std::string end_effector;
        } prbt_link_names;

        std::string manipulator_name;
	    std::vector<std::string> manipulator_joint_names;
        std::vector<float> prbt_joint_limits = {2.967, 2.095, 2.356, 2.967, 2.966, 3.123};
        std::vector<double> home_position = {0.08, 0.75, 2.35, 1.55, 0.00, -0.241};
        
        trajectory_msgs::JointTrajectory prbt_trajectory;

		dynamic_planner *dyn_planner;

        moveit_msgs::CollisionObject mobile_base, floor, controller_cabinet;

        void Obstacle_Pose (moveit_msgs::CollisionObject *object, std::string reference_id, std::string object_id, float position[3], float orientation[4]);
        void Obstacle_Dimension (moveit_msgs::CollisionObject *object, float x, float y, float z);

        void plan_joint_space(std::vector<double> final_position, bool send = true);
		void plan_cartesian_space(geometry_msgs::PoseStamped final_pose, std::string link_name, bool send = true);

        void set_planner_parameters(std::string planner_id = "", int attempts = 0, double time = 0, double v_factor = 99.0, double a_factor = 99.0);
		void set_planner_dynamic(double v_factor = 99.0, double a_factor = 99.0);

        geometry_msgs::Pose compute_end_effector_position (void);

};

#endif /* PRBT_PLANNER_H */