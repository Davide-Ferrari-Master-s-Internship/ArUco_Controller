#include "aruco_alignment.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ArUco_Alignment");

    aruco_alignment aa;

    while (ros::ok()) {

        aa.align();
        ros::shutdown();

    }

return 0;

}
