#include <ros/ros.h>
#include "reltive_move.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "reltive_move");
    
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
