#include "dynamic_planner/dynamic_planner.h"

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "dynamic_planner");

	ros::AsyncSpinner spinner(1);
	spinner.start();
	std::string a = "manipulator";
	
	std::vector<std::string>  j = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
	// std::vector<std::string>  j = {"prbt_joint_1", "prbt_joint_2", "prbt_joint_3", "prbt_joint_4", "prbt_joint_5", "prbt_joint_6"};
	std::vector<double> i(6);
	
	i = {0.0,0.0,0.0,0.0,0.0,0.0};
	
	dynamic_planner ce(a,j,i);
	
	int counter = 0;
	std::vector<double> final_position(6);
	int point_executed;

	while(ros::ok())
	{
		if(counter == 0){
			ce.visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

			final_position = {1.57, -1.57, 1.57, -1.57, -1.57, -1.57}; 
			counter = 1;
		} else if (counter == 1) {
			final_position = {0.7, -1.2, 1.57, -1.9, -1.57, -2.45};
			counter = 2;
		} else {
			final_position = {2.3, -1.2, 1.57, -1.9, -1.57, -2.45};
			counter = 1;

		}
    
    	ce.plan(final_position, true);
		// ce.check_trajectory(ce.trajectory, point_executed);
		ros::shutdown();
		
		// ce.spinner();

	}

return 0;

}
