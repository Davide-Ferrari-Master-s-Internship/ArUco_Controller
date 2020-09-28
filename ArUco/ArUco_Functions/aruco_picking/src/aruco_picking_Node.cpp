#include "aruco_picking.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ArUco_Picking");

    aruco_picking ap;

    while (ros::ok()) {

        ap.pick();
        ros::shutdown();

    }

return 0;

}
