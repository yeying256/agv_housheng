#include <ros/ros.h>
#include "reltive_move.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "visual_reltivmove");
    ros::AsyncSpinner s(3);
    s.start();
    xj_control_ns::reltive_move reltive_move;
    reltive_move.move_visual();
    s.stop();
    return 0;
}
