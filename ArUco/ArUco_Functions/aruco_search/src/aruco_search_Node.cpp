#include "aruco_search.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ArUco_Search");

    aruco_search as;

    while (ros::ok()) {

        as.search();
        ros::shutdown();

    }

return 0;

}
