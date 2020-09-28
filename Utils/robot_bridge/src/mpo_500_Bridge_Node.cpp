#include "mpo_500_Bridge.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "mpo_500_Bridge");

    mpo_500_bridge mpo;

    while (ros::ok()) {

        mpo.spinner();

    }

return 0;

}
