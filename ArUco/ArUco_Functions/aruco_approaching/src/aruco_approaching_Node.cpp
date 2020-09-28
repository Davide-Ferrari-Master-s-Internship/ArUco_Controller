#include "aruco_approaching.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ArUco_Approaching");

    aruco_approaching aa;

    while (ros::ok()) {

        aa.approach();
        ros::shutdown();

    }

return 0;

}
