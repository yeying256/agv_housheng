#include <ros/ros.h>
#include "reltive_move.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "visual_relative_point_move");
    ros::AsyncSpinner s(3);
    s.start();
    xj_control_ns::reltive_move reltive_move;
    reltive_move.init(xj_control_ns::reltive_move::Visual_Mode::POINT_MOVE);
    reltive_move.run();
    s.stop();
    return 0;
}
