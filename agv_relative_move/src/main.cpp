#include <ros/ros.h>
#include "reltive_move.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "reltive_move");
    xj_control_ns::reltive_move reltive_move;
    ros::MultiThreadedSpinner s(2);
    // ros::AsyncSpinner spinner(3);
    // spinner.start();
    // ros::waitForShutdown();
    // spinner.stop();

    ros::spin(s);
    return 0;
}
