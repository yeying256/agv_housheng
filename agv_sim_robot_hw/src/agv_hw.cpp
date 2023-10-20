#include "agv_hw.h"
#include "stdio.h"
#include "zmotion.h"
#include "zmcaux.h"
#include "ros/ros.h"
#include "agv_msg/grab_agv.h"



namespace xj_control_ns
{
    Agv_hw_interface::Agv_hw_interface(/* args */)
    {
        handle = NULL;
    }
    
    Agv_hw_interface::~Agv_hw_interface()
    {
        for (int i = 0; i < 4; i++)
        {
            ZAux_Direct_Single_Cancel(handle,i,2);
        }
        
        
    }

    void Agv_hw_interface::commandCheckHandler(const char *command, int ret)//检查指令执行结果
    {
        if (ret)//非 0 则失败
        {   
            printf("%s return code is %d\n", command, ret);
        }   
    }   
    
    //夹爪服务器
    bool Agv_hw_interface::Grab_Server(agv_msg::grab_agv::Request &req,
                    agv_msg::grab_agv::Request &resp){
        float high_unit = req.high*(625000/471);//将几何参数转换为用户单位输入 10000pulse/r
        float width_unit = req.width*(2000/3);//将几何参数转换为用户单位输入
        this->status_ = req.status;
        ROS_INFO("夹爪收到的参数为:high=%.2f,width=%.2f,status_ = %d",req.high,req.width,status_);
        // ZMC_HANDLE handle = NULL;
        uint32 homestatus; 
        
        for (int i = 0; i < 4; i++)
        {
            ZAux_Direct_SetAxisEnable(handle,i,1);

        }
        

        if((status_ == 3)||(status_ ==5)){
            ROS_INFO("夹爪处于运动状态");
            ZAux_Direct_Single_MoveAbs(handle,4, high_unit);//上升电机同步运动
            ZAux_Direct_Single_MoveAbs(handle,7, width_unit);//夹取运动电机运动
            ZAux_Direct_SetAtype(handle, 0, 66);
            ZAux_Direct_SetAtype(handle, 2, 66);  
        }
        else if(status_ == 1){//AGV回零
            ZAux_Direct_SetAtype(handle, 0, 65);
            ZAux_Direct_SetAtype(handle, 2, 65);             
            ZAux_Direct_SetSpeed(handle, 0, 20000);
            ZAux_Direct_SetSpeed(handle, 2, 20000);
            ZAux_Direct_SetCreep(handle, 0, 1000);
            ZAux_Direct_SetCreep(handle, 2, 1000);
            // ZAux_Direct_SetInvertIn(handle, i, 1);//设置需要回零的输入口电平反转
            // ZAux_Direct_SetDpos( handle, i, 0);//设置需要回零的轴指令位置清0
            // ZAux_Direct_SetMpos( handle,i, 0);//设置需要回零的轴反馈位置清0
            ZAux_Direct_SetAxisEnable(handle,1,0);
            int retBD0 = ZAux_BusCmd_Datum(handle, 0,3);
            printf("*************是否回零，%d***************\n",retBD0);

            while (ros::ok())//等待轴 0 回零运动完成
            {
                sleep(1);
                ZAux_Direct_GetHomeStatus(handle,0,&homestatus);//获取回零运动完成状态
                if (homestatus==1){
                printf("*************轴0回零了***************\n");
                    break;}
                printf("*************轴0正在回零***************\n");

            }
            commandCheckHandler("ZAux_BusCmd_Datum", retBD0);
            ZAux_Direct_SetAxisEnable(handle,1,1);
            ZAux_Direct_SetAxisEnable(handle,3,0);
            int retBD2 = ZAux_BusCmd_Datum(handle, 2,3);
            while (ros::ok())//等待轴 2 回零运动完成
            {
                sleep(1);
                ZAux_Direct_GetHomeStatus(handle,2,&homestatus);//获取回零运动完成状态
    
                    if (homestatus==1){
                printf("*************轴2回零了***************\n");
                    break;}
                printf("*************轴2正在回零***************\n");
                
            }
            commandCheckHandler("ZAux_BusCmd_Datum", retBD2);
            ZAux_Direct_SetAxisEnable(handle,3,1);
            ZAux_Direct_SetAtype(handle, 0, 66);
            ZAux_Direct_SetAtype(handle, 2, 66);  
        }    
        else if(status_==2){//夹爪回零
            int retBD4 = ZAux_BusCmd_Datum(handle, 4,3);while (1)//等待轴 4 回零运动完成
            {
                sleep(1);
                ZAux_Direct_GetHomeStatus(handle,0,&homestatus);//获取回零运动完成状态
                if (homestatus==1){break;}
            }
            commandCheckHandler("ZAux_BusCmd_Datum", retBD4);
            int retBD6 = ZAux_BusCmd_Datum(handle, 4,3);while (1)//等待轴 6 回零运动完成
            {
                sleep(1);
                ZAux_Direct_GetHomeStatus(handle,0,&homestatus);//获取回零运动完成状态
                if (homestatus==1){break;}
            }
            commandCheckHandler("ZAux_BusCmd_Datum", retBD6);
        }
        else{
            ROS_INFO("夹爪处于静止状态");
        }
        return true;
    }

    bool Agv_hw_interface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
    {
        setlocale(LC_ALL,"");
        bool ret = robot_hw_nh.getParam("/agv_sim/robot_hw_test/joints", agv_joint_name);//从参数服务器中获取name
        if (ret == true)
        {
            std::cout<<"\033[1;32;40m 获得jointname成功\033[0m"<<std::endl;
        }else
        {
            std::cout<<"\033[1;31;40m 获得jointname失败\033[0m"<<std::endl;
        }
        
        ROS_ERROR("getParam ret= %d",ret);
        
        agv_num_joints_ = agv_joint_name.size();

        ROS_ERROR("agv_joint_name.size()= %d",agv_joint_name.size());


        for (size_t i = 0; i < agv_num_joints_; i++)
        {
            ROS_ERROR("jointname=%s",agv_joint_name[i].c_str());
        }
        joint_effort_state.resize(agv_num_joints_);
        joint_position_state.resize(agv_num_joints_);
        joint_velocity_state.resize(agv_num_joints_);
        joint_velocity_command.resize(agv_num_joints_);


        for(int i=0; i<agv_num_joints_; i++)
        {
            //State
            hardware_interface::JointStateHandle jointStateHandle(agv_joint_name[i], &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
            joint_state_interface.registerHandle(jointStateHandle);
            ROS_INFO("joint_name[%d].c_str()=%s",i,jointStateHandle.getName());

            //Velocity
            hardware_interface::JointHandle jointVelocityHandle(joint_state_interface.getHandle(agv_joint_name[i]), &joint_velocity_command[i]);
            velocity_joint_interface.registerHandle(jointVelocityHandle);
        }

        registerInterface(&joint_state_interface);          //将类中的接口注册到ros中
        registerInterface(&velocity_joint_interface);    

//****************************************************************************************************
        //PCI初始化

            uint32 cardnumcardnum = 0;
            //连接返回的句柄
            //PCI接口编号
            int retPCI = ZAux_OpenPci(cardnumcardnum, &handle); ////以太网连接控制器以太网连接控制器
            commandCheckHandler("ZAux_OpenPci", retPCI);
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
            //do something.....
        

//*********************************************************************************************************
        int AxisEnable;
        for(int axis=0;axis<4; axis++)
        {
        ZAux_BusCmd_DriveClear(handle,axis,0);//0-清除当前 1-清除历史 2-清除外部输入警告


                int retSAE = ZAux_Direct_SetAxisEnable(handle, axis, 1);// 设置轴使能： 0 表示关闭， 1 表示打开
                commandCheckHandler("ZAux_Direct_SetAxisEnable", retSAE);
                if (retSAE == 0)
                {
                    printf("打开使能\n");
                }
                
        }

        //机器人参数初始化********************************************************************************************
        for(int i=0;i < 4;i++)
        {
        
            if(i<4){//AGV电机参数初始化
                ZAux_Direct_SetAtype(handle, i, 66);//设置各轴的类型为66（EtherCAT总线周期速度模式）
            }
        //夹爪参数设置
            // else if(i==4){//夹爪上升主动电机参数初始化
            //     ZAux_Direct_SetAtype(handle, i, 65);//设置各轴的类型为65（EtherCAT总线周期位置模式）
            //     ZAux_Direct_SetUnits(handle, i, 625000/471); //设置各轴脉冲当量为 100
            //     ZAux_Direct_SetSpeed(handle, i, 100); //设置轴 0 速度为 200units/s
            //     ZAux_Direct_SetAccel(handle, i, 1000); //设置各轴加速度为 2000units/s/s
            //     ZAux_Direct_SetDecel(handle, i, 1000); //设置各轴减速度为 2000units/s/s
            //     // ZAux_Direct_SetHomeWait(handle, i, 1000);//设置各轴回零等待时间
            //     ZAux_Direct_SetFastDec(handle, i, 3000) ;//设置转向急停时快减速度为 3000units/s/s
            //     ZAux_Direct_SetSramp(handle, i, 200); //设置各轴S曲线时间为 200ms
            //     ZAux_Direct_SetCreep(handle, i, 50);//设置回零时反向爬行速度
            // }
            // else if(i==5){//夹爪上升从动电机参数初始化
            //     ZAux_Direct_SetAtype(handle, i, 65);//设置各轴的类型为65（EtherCAT总线周期位置模式）
            //     ZAux_Direct_SetUnits(handle, i, 100); //设置各轴脉冲当量为 100
            // }
            // else{//夹爪开合电机参数初始化
            //     ZAux_Direct_SetAtype(handle, i, 65);//设置各轴的类型为65（EtherCAT总线周期位置模式）
            //     ZAux_Direct_SetUnits(handle, i, 2000/3); //设置各轴脉冲当量为 666.66667
            //     ZAux_Direct_SetSpeed(handle, i, 100); //设置轴 0 速度为 200units/s
            //     ZAux_Direct_SetAccel(handle, i, 1000); //设置各轴加速度为 2000units/s/s
            //     ZAux_Direct_SetDecel(handle, i, 1000); //设置各轴减速度为 2000units/s/s
            //     // ZAux_Direct_SetHomeWait(handle, i, 1000);//设置各轴回零等待时间
            //     ZAux_Direct_SetFastDec(handle, i, 3000) ;//设置转向急停时快减速度为 3000units/s/s
            //     ZAux_Direct_SetSramp(handle, i, 200); //设置各轴S曲线时间为 200ms
            //     ZAux_Direct_SetCreep(handle, i, 50);//设置回零时反向爬行速度
            // }
            // ZAux_Direct_Single_Addax(handle,5,4);
        }
    

        
        // robot_hw_nh.advertiseService("robot_control",Grab_Server);//启动夹爪运动服务器
        status_ = 0;

        command_service_ = root_nh.advertiseService("/robot_control",&Agv_hw_interface::Grab_Server,this);//启动夹爪运动服务器

        printf("夹爪机器人服务器已启动！");   
        // ros::spin();
        return true;
    }

    void Agv_hw_interface::read(const ros::Time& time, const ros::Duration& period)
    {
        setlocale(LC_ALL,"");
        float velocity_unit;
        float position_unit;
        for(int i=0;i<4;i++)
        {
            int retGS =ZAux_Direct_GetMspeed(handle, i, &velocity_unit); //获取轴的速度(unit/s)
            int retGMPOS = ZAux_Direct_GetMpos(handle, i, &position_unit);//获取反馈位置(unit)
            commandCheckHandler("ZAux_Direct_GetMspeed", retGS);//判断指令是否执行成功
            commandCheckHandler("ZAux_Direct_GetMpos", retGMPOS) ;//判断指令是否执行成功
            if(i==0||i==2){
                joint_velocity_state[i]=velocity_unit/(10000*20/3.14159/2);//将用户单位速度转换为电机转速
                joint_position_state[i]=position_unit/(10000*20/3.14159/2);//将用户单位位置转换为几何位置
                printf("轴%d的速度 Speed = %lfrad/s\n", i, joint_velocity_state[i]);
                
            }
            else{
                joint_velocity_state[i]=velocity_unit/(10000*17/3.14159/2);//将用户单位速度转换为电机转速
                // printf("轴%d的速度 Speed = %lf\n", i, joint_velocity_state[i]);
            }
        }

    }
    void Agv_hw_interface::write(const ros::Time& time, const ros::Duration& period)
    {
        setlocale(LC_ALL,"");
        // ZMC_HANDLE handle = NULL;
        float DAC[4];

        // for (int i = 0; i < 4; i++)
        // {
        //     std::cout<<"\033[1;36;40m "<<"joint_velocity_command"<<i<<"="<<joint_velocity_command[i]<<"\033[0m "<<std::endl;
        // }

        // printf("agv_num_joints_ = %d,\n",agv_num_joints_);
        
        // status_ =100;

        switch (status_)
        {
        case 0:
            ROS_INFO("***急停状态！***");
            for(int i=0; i<agv_num_joints_; i++){
                int retSVS = ZAux_Direct_SetDAC(handle, i, 0);
                commandCheckHandler("ZAux_Direct_SetDAC", retSVS);
            }
            for(int i=4; i<7;i++){
                int retSVS = ZAux_Direct_Single_Cancel(handle, i, 2);
                commandCheckHandler("ZAux_Direct_SetDAC", retSVS);
            }
            break;
        case 1:
            // ROS_INFO("***AGV回零***");
            break;
        case 2:
            ROS_INFO("***夹爪回零***");
            break;
        case 3:
            ROS_INFO("***夹爪与底盘共同运动***");        
            for(int i=0;i<agv_num_joints_; i++){
                if(i==0||i==2){
                    DAC[i]=joint_velocity_command[i]*10000*20/3.14159/2;//转向电机用户单位速度转换为电机转速
                    int retSD = ZAux_Direct_SetDAC(handle, i, DAC[i]);
                    commandCheckHandler("ZAux_Direct_SetDAC", retSD);
                }
                else{
                    DAC[i]=joint_velocity_command[i]*10000*17/3.14159/2;//行走电机用户单位速度转换为电机转速
                    int retSD = ZAux_Direct_SetDAC(handle, i, DAC[i]);
                    commandCheckHandler("ZAux_Direct_SetDAC", retSD);
                }
            }
            break;
        case 4:
            ROS_INFO("***底盘单独运动***");
            for(int i=0; i<agv_num_joints_; i++){
                if(i==0||i==2){
                    DAC[i]=joint_velocity_command[i]*10000*20/3.14159/2;//转向电机用户单位速度转换为电机转速 dac = 速度（rad/s）/（2*pi）*10000（pulse/r）*20（减速比）
                    int retSD = ZAux_Direct_SetDAC(handle, i, DAC[i]);
                    commandCheckHandler("ZAux_Direct_SetDAC", retSD);
                    float position;
                    ZAux_Direct_GetMpos(handle,2,&position);
                    if(i==2){
                        printf("轴2位置：%.2f",position);
                    }

                }
                else{
                    DAC[i]=joint_velocity_command[i]*10000*17/3.14159/2;//行走电机用户单位速度转换为电机转速
                    int retSD = ZAux_Direct_SetDAC(handle, i, DAC[i]);
                    commandCheckHandler("ZAux_Direct_SetDAC", retSD);
                }

            // std::cout<<"\033[1;36;40m "<<"DAC"<<i<<"="<<DAC[i]<<"\033[0m "<<std::endl;

            }
            break;
        case 5:
            ROS_INFO("***夹爪单独运动***");        
            for(int i=0; i<agv_num_joints_; i++){
                int retSVS = ZAux_Direct_SetDAC(handle, i, 0);
                commandCheckHandler("ZAux_Direct_SetDAC", retSVS);
            }
            break;
        case 100:
            {
                // ZAux_Direct_SetAtype(handle,0,66);
                ZAux_Direct_SetDAC(handle, 0, 10000);
                break;
            }
        default:
            break;
        }
    }

}
PLUGINLIB_EXPORT_CLASS(xj_control_ns::Agv_hw_interface, hardware_interface::RobotHW)