#include "aruco_controller.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ArUco_Controller");

    aruco_controller ac;

    while (ros::ok()) {

        ac.spinner();

    }

return 0;

}
