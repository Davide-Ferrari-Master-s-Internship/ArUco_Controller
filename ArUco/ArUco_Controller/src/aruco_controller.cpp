#include "aruco_controller.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//


aruco_controller::aruco_controller() {

    
}


aruco_controller::~aruco_controller() {

    
}


//-------------------------------------------------------- MAIN --------------------------------------------------------//


void aruco_controller::spinner (void) {
    
    //wait for planner instantiation
    ros::Duration(3).sleep();
    ros::spinOnce();

SEARCH:

    ROS_WARN("Starting ArUco Search");

    ar_search = new aruco_search();
    ar_search -> move_to_search_point();
    ar_search -> search();

ALIGN:

    if (ar_search -> aruco_found) {

        ROS_WARN("Starting ArUco Alignment");

        delete ar_search;

        // ros::Duration(2).sleep();

        ar_alignment = new aruco_alignment();
        ar_alignment -> align();

        if (!ar_alignment -> aruco_found) {delete ar_alignment; goto SEARCH;}
        if (!ar_alignment -> aruco_aligned) {delete ar_alignment; goto ALIGN;}

    }

APPROACH:

    if (ar_alignment -> aruco_aligned) {

        ROS_WARN("Starting ArUco Approaching");

        delete ar_alignment;

        // ros::Duration(2).sleep();
        // ros::Duration(10).sleep();

        ar_approaching = new aruco_approaching();
        ar_approaching -> approach();

        if (!ar_approaching -> aruco_found) {delete ar_approaching; goto SEARCH;}
        if (!ar_approaching -> aruco_aligned) {delete ar_approaching; goto ALIGN;}
        if (!ar_approaching -> aruco_approached) {delete ar_approaching; goto APPROACH;}

    }

PICK:

    if (ar_approaching -> aruco_approached) {

        ROS_WARN("Starting ArUco Picking");
        
        delete ar_approaching;

        // ros::Duration(2).sleep();
        // ros::Duration(10).sleep();

        ar_picking = new aruco_picking();
        ar_picking -> pick();

        if (!ar_picking -> aruco_found) {
            
            delete ar_picking;

            // ros::Duration(2).sleep();
            // ros::Duration(10).sleep();

            ROS_WARN("Starting ArUco Search");

            ar_search = new aruco_search();
            ar_search -> search();
            
            if (!ar_search -> aruco_found) {ROS_ERROR("Cannot Find ArUco Marker, Shutdown"); goto END;}
            else {

                delete ar_search;

                // ros::Duration(2).sleep();
                // ros::Duration(10).sleep();

                ROS_WARN("Starting ArUco Alignment");

                ar_alignment = new aruco_alignment();
                ar_alignment -> align_manipulator_only();

                if (!ar_alignment -> aruco_aligned) {ROS_ERROR("Cannot Align ArUco Marker, Shutdown"); goto END;}
                else {
                
                    delete ar_alignment;

                    ROS_WARN("Starting ArUco Picking");
                    
                    ar_picking = new aruco_picking();
                    ar_picking -> pick();

                }

            }

        }
        
        // if (!ar_picking -> picking_complete) {delete ar_picking; goto PICK;}

    }

END:

    ros::shutdown();

}
