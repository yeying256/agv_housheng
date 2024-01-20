#include "ros/ros.h"
#include "grab_test/test.h"
#include "grab_test/zmotion.h"
#include <grab_test/zmcaux.h>


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Test_Client");
    ros::NodeHandle nh;
    ros::ServiceClient client=nh.serviceClient<grab_test::test>("Test");
    grab_test::test tt;
    tt.request.start=atoi(argv[1]);
    //客户端创建完毕之后，访问服务器之前挂起
    client.waitForExistence();
    bool flag=client.call(tt);
    if(flag){
        ROS_INFO("处理成功");
    }
    else{
        ROS_INFO("处理失败");
    }
    return 0;
}
