#include "ros/ros.h"
#include "grab_test/test.h"
#include "grab_test/zmotion.h"
#include <grab_test/zmcaux.h>

void commandCheckHandler(const char *command, int ret) // 检查指令执行结果
{
    if (ret) // 非 0 则失败
    {
        ROS_INFO("%s return code is %d\n", command, ret);
    }
}

bool motorMove(grab_test::test::Request &req,
               grab_test::test::Response &resp)
{
    int start = req.start;
    int result = resp.result;
    int successss;
    ZMC_HANDLE handle = NULL;
    uint32 cardnumcardnum = 0;
    uint32 homestatus;
    int a = 0;

    // 连接返回的句柄
    // PCI0
    //  ZMC_Open()
    // do something.....

    

    // 逻辑处理

    int retPCI = ZAux_OpenPci(0, &handle); ////以太网连接控制器以太网连接控制器
    // commandCheckHandler("ZAux_OpenPci", retPCI);
    if (ERR_SUCCESS != retPCI)
    {
        printf("PCI 连接失败!\n");
        handle = NULL;
        return 0;
    }
    else
    {
        printf("PCI 连接成功!\n");
    }
            ZAux_BusCmd_DriveClear(handle, a, 0); // 0-清除当前 1-清除历史 2-清除外部输入警告

        int retSAE = ZAux_Direct_SetAxisEnable(handle, a, 1); // 设置轴使能： 0 表示关闭， 1 表示打开;
        // sleep(1);
        commandCheckHandler("ZAux_Direct_SetAxisEnable", retSAE);
        if (retSAE == 0)
        {
            printf("打开使能\n");
        }
    // ZAux_Direct_SetAxisEnable(handle, 3, 1);
    if (start == 1)
    {
        int axisstatus;
        float open_position;
        // int b = 2;
        // for(int a=1;a<3;a++){
        // 夹爪上升主动电机参数初始化
        int retsa = ZAux_Direct_SetAtype(handle, a, 65);         // 设置各轴的类型为65（EtherCAT总线周期位置模式）
        commandCheckHandler("retsaZAux_Direct_SetAtype", retsa);

        ZAux_Direct_SetUnits(handle, a, 10000 / 15); // 设置各轴脉冲当量为 100
        // ZAux_Direct_SetUnits(handle, a, 200000/360); // 设置各轴脉冲当量为 100

        // ZAux_Direct_SetAtype(handle, b, 65);            // 设置各轴的类型为65（EtherCAT总线周期位置模式）
        // ZAux_Direct_SetUnits(handle, b, 625000 / 471);  // 设置各轴脉冲当量为 100
        ZAux_Direct_SetSpeed(handle, a, 30);   // 设置轴 0 速度为 200units/s
        ZAux_Direct_SetAccel(handle, a, 1000); // 设置各轴加速度为 2000units/s/s
        ZAux_Direct_SetDecel(handle, a, 1000); // 设置各轴减速度为 2000units/s/s
        // ZAux_Direct_SetHomeWait(handle, a, 1000);    //设置各轴回零等待时间
        ZAux_Direct_SetFastDec(handle, a, 3000); // 设置转向急停时快减速度为 3000units/s/s
        ZAux_Direct_SetSramp(handle, a, 200);    // 设置各轴S曲线时间为 200ms
        ZAux_Direct_SetCreep(handle, a, 50);     // 设置回零时反向爬行速度
        ZAux_Direct_SetFwdIn(handle, a, 129);    // 设置电机正限位开关IO
        ZAux_Direct_SetRevIn(handle, a, 128);    // 设置电机负限位开关IO
        std::cout<<"回零点开始"<<std::endl;
        int fuck = ZAux_Direct_Single_Vmove(handle, a, -1);
        std::cout<<"回零返回值="<< fuck<<std::endl;

        while (ros::ok())
        {
            ZAux_Direct_GetAxisStatus(handle, a, &axisstatus);
            
            if (axisstatus == 32)
            {
                ZAux_Direct_SetMpos(handle, a, 0);
                break;
            }
            // int fuck1;
            sleep(1);

        //     ZAux_Direct_GetAxisEnable(handle,a,&fuck1);
        // std::cout<<"查询状态,是否使能 fuck1="<<fuck1<<std::endl;

        }
        sleep(1);
        ZAux_Direct_Single_MoveAbs(handle, a, 280);

        while (ros::ok()){
            sleep(0.1);
            ZAux_Direct_GetIfIdle(handle,a,&successss);
            ZAux_Direct_GetMpos(handle, a , &open_position);
            if (successss == -1){
                break;
            }
            printf("open_position=%f\n",open_position);        int a = 5;
            
        }


        // // printf("运动函数执行结果：%d",ret);
        // sleep(40);
        // ZAux_Direct_SetAxisEnable(handle, a,0);
        // ZAux_BusCmd_Datum(handle,a, 22);
        // int b = 1;
        // // 夹爪上升从动电机参数初始化
        // ZAux_Direct_SetAtype(handle, b, 65);            // 设置各轴的类型为65（EtherCAT总线周期位置模式）
        // ZAux_Direct_SetUnits(handle, b, 625000 / 471);  // 设置各轴脉冲当量为 100
        // ZAux_Direct_Single_Addax(handle,b,a);
        // ZAux_Direct_Single_Vmove(handle,a,-1);
        // ZAux_Direct_SetAxisEnable(handle, 4, 0);
        // ZAux_Direct_SetAxisEnable(handle, 3, 0);
        // ZAux_Direct_SetAxisEnable(handle, 6, 0);

        // int retBD0 = ZAux_BusCmd_Datum(handle, a, 22);
        //     commandCheckHandler("ZAux_BusCmd_Datum", retBD0);
        //     // 前轮转向电机回零运动：0->FRONT_STEER
        //     if (!retBD0)
        //     {
        //         while (ros::ok())
        //         {
        //             sleep(1);
        //             ZAux_Direct_GetHomeStatus(handle, a, &homestatus); // 获取回零运动完成状态
        //             // float tmp_position;
        //             // ZAux_Direct_GetMpos(handle,FRONT_STEER,&tmp_position);
        //             // int tmp_enable;
        //             // ZAux_Direct_GetAxisEnable(handle,FRONT_STEER,&tmp_enable);
                    
        //             // ROS_INFO("***position FRONT_STEER =%f***\n",tmp_position);
        //             // ROS_INFO("***enable FRONT_STEER =%d***\n",tmp_enable); //0是失败 1成功


        //             if (homestatus == 1)
        //             {
        //                 ROS_INFO("***前转向轮回零了***\n");
        //                 break;
        //             }
        //             ROS_INFO("***前转向轮正在回零***\n");
        //         }
        //     }
            // ZAux_Direct_SetMpos(handle, a, 0);
            sleep(100);
        ZAux_Direct_SetAxisEnable(handle, a, 0);
        // ZAux_Direct_SetAxisEnable(handle, 3, 0);
        // ZAux_Direct_SetAxisEnable(handle, 4, 0);
        // ZAux_Direct_SetAxisEnable(handle, 6, 0);

        return true;
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // ZMC_HANDLE handle = NULL;
    // unit32 cardnumcardnum=0;

    // 2.初始化 ROS 节点
    ros::init(argc, argv, "Test_Server");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建 服务 对象
    ros::ServiceServer server = nh.advertiseService("Test", motorMove);
    ROS_INFO("服务已经启动....");
    //     5.回调函数处理请求并产生响应
    //     6.由于请求有多个，需要调用 ros::spin()
    ros::spin();

    return 0;
}
