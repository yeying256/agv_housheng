#include <ros/ros.h>
#include "reltive_move.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "reltive_move");


    ros::AsyncSpinner s(5);
    // ros::MultiThreadedSpinner s(2);
    // ros::AsyncSpinner spinner(3);
    // spinner.start();
    s.start();
    xj_control_ns::reltive_move reltive_move;
    reltive_move.move();
    s.stop();
    // ros::waitForShutdown();
    // spinner.stop();
    // ros::spin(s);
    return 0;
}
