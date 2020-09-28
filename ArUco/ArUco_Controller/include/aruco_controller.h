#ifndef ARUCO_CONTROLLER_H
#define ARUCO_CONTROLLER_H

#include "ros/ros.h"

#include "aruco_search.h"
#include "aruco_alignment.h"
#include "aruco_approaching.h"
#include "aruco_picking.h"

class aruco_controller {

    public:

        aruco_controller();
        ~aruco_controller();

        void spinner (void);

    private:

        ros::NodeHandle nh;

		aruco_search *ar_search;
        aruco_alignment *ar_alignment;
        aruco_approaching *ar_approaching;
        aruco_picking *ar_picking;

};

#endif /* ARUCO_CONTROLLER_H */