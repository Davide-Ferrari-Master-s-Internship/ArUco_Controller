#include "Robot_Bridge.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "Robot_Bridge");

    bridge rb;

    while (ros::ok()) {

        rb.spinner();

    }

return 0;

}
