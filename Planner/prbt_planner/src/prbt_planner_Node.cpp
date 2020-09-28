#include "prbt_planner.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "prbt_planner.h");

    prbt_planner_class p;

    while (ros::ok()) {

        p.spinner();

    }

    ros::shutdown();

return 0;

}
