#include "ArUco_Detection/ArUco_Detection.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "ArUco_Detection");

    ArUco_Detection ad;

    while (ros::ok()) {

        ad.spinner();

    }

return 0;

}
