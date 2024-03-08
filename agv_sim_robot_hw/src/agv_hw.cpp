#include "agv_hw.h"
#include "stdio.h"
#include <unistd.h>
#include "ros/ros.h"
#include "agv_msg/grab_agv.h"
#include "agv_msg/hw_srv.h"
#include "agv_msg/error_log.h"

namespace xj_control_ns
{
    Agv_hw_interface::Agv_hw_interface() : handle(NULL)
    {
        log_pub_ = nh.advertise<agv_msg::error_log>("/hardware_logger", 1);
    }

    Agv_hw_interface::~Agv_hw_interface()
    {
        for (int i = GRAB_OPEN; i <= BACK_WHEEL_2; i++)
        {
            int tmp;
            ZAux_Direct_GetIfIdle(handle, i, &tmp);
            if (tmp == -1)
            {
                ZAux_Direct_Single_Cancel(handle, i, 2);
            }
        }
    }

    void Agv_hw_interface::commandCheckHandler(const char *command, int ret) // 检查指令执行结果
    {
        if (ret) // 非 0 则失败
        {
            ROS_INFO("%s return code is %d\n", command, ret);
        }
    }

    // 夹爪服务器
    bool Agv_hw_interface::Grab_Server(agv_msg::grab_agv::Request &req,
                                       agv_msg::grab_agv::Response &resp)
    {
        // float high_unit = req.high * (625000 / 471); // 将几何参数转换为用户单位输入 10000(pulse/r)*25/3.14/60
        // float high_unit = req.high; // 将几何参数转换为用户单位输入 10000(pulse/r)*25/3.14/60
        // float width_unit = req.width;
        // float width_unit = req.width * (10000 / 15); // 将几何参数转换为用户单位输入 10000(pulse/r)/15(mm/r)
        this->status_ = req.status;
        // ROS_INFO("夹爪收到的参数为:high=%.2f,width=%.2f,status_ = %d", req.high, req.width, status_);
        uint32 homestatus;
        int axisstatus;
        float open_pos;
        float lift_pos;

        // int32 status #0:急停；1：AGV回零；2：夹爪回零；3：两部分一起运动；4：AGV单独运动；5：夹爪单独运动
        // switch (req.status)
        // {
        // case 0:
        //     ROS_INFO("急停模式");
        //     break;
        // case 1:
        //     ROS_INFO("AGV回零");
        //     break;
        // case 2:
        //     ROS_INFO("夹爪回零");
        //     break;
        // case 3:
        //     ROS_INFO("两部分一起运动");
        //     break;
        // case 4:
        //     ROS_INFO("AGV单独运动");
        //     break;
        // case 5:
        //     ROS_INFO("夹爪单独运动");
        //     break;
        // default:
        //     ROS_INFO("参数错误");
        //     break;
        // }
        // if (req.status == 2)
        // { // 夹爪回零 4->GRAB_OPEN
        //     // resp.agv_result = 0;
        //     printf("准备开始");
        //     // int retGGH = ZAux_Direct_Single_Vmove(handle, GRAB_OPEN, -1);
        //     int retGGH = 0;
        //     sleep(5);
        //     printf("回零结果：%d", retGGH);
        //     // while (ros::ok()) // 等待轴 4 回零运动完成
        //     // {
        //     //     // sleep(1);
        //     //     ZAux_Direct_GetAxisStatus(handle, GRAB_OPEN, &axisstatus); // 获取回零运动完成状态
        //     //     if (axisstatus == 32)
        //     //     {
        //     //         // sleep(1);
        //     //         ZAux_Direct_SetMpos(handle, GRAB_OPEN, 0);
        //     //         // sleep(0.5);
        //     //         ROS_INFO("***夹爪开合电机回零了***\n");
        //     //         break;
        //     //     }
        //     // }
        //     sleep(1);
        //     // return 1;

        // }

        if ((status_ == 3) || (status_ == 5))
        {
            ROS_INFO("夹爪处于运动状态");
            int state[2];
            resp.grab_result = 0;
            int tmp = 0;
            if (req.width < 900.0)
            {
                ZAux_Direct_Single_MoveAbs(handle, GRAB_OPEN, req.width); // 夹取运动电机运动
                // printf("夹爪开合接受的指令：%f\n", req.width);
                while (ros::ok() && tmp < 20)
                {
                    tmp++;
                    ZAux_Direct_GetMpos(handle, GRAB_OPEN, &open_pos);
                    // printf("夹爪开合的位置：%f\n", open_pos);
                    ZAux_Direct_GetIfIdle(handle, GRAB_OPEN, state);
                    if (state[0] == -1)
                    {
                        break;
                    }
                    sleep(1);
                }
                if (tmp == 20)
                    ROS_ERROR("Grab Open Failed");
            }
            else
                ROS_ERROR("Data Error: invalid grab_width");

            sleep(0.5);
            tmp = 0;
            if (req.high < 900.0)
            {
                ZAux_Direct_Single_MoveAbs(handle, GRAB_LEFT, req.high); // 上升电机同步运动
                while (ros::ok() && tmp < 20)
                {
                    tmp++;
                    printf("夹爪上升位置：%f\n", lift_pos);
                    ZAux_Direct_GetIfIdle(handle, GRAB_LEFT, state + 1);
                    if (state[1] == -1)
                    {
                        resp.grab_result = 1;
                        break;
                    }
                    sleep(1);
                }
                if (tmp == 20)
                    ROS_ERROR("Grab Lift Failed");
            }
            else
                ROS_ERROR("Data Error: invalid grab_height");

            printf("***夹爪定位完成***\n");
        }

        if (status_ == 1)
        {
            resp.agv_result = 0;

            ZAux_Direct_SetAtype(handle, FRONT_STEER, 65);
            ZAux_Direct_SetAtype(handle, BACK_STEER, 65);
            ZAux_Direct_SetSpeed(handle, FRONT_STEER, 72);
            ZAux_Direct_SetSpeed(handle, BACK_STEER, 72);
            ZAux_Direct_SetCreep(handle, FRONT_STEER, 36);
            ZAux_Direct_SetCreep(handle, BACK_STEER, 36);

            // 前轮转向电机回零运动
            ZAux_Direct_SetAxisEnable(handle, FRONT_WHEEL, 0);
            ZAux_Direct_SetAxisEnable(handle, BACK_STEER, 0);
            ZAux_Direct_SetAxisEnable(handle, BACK_WHEEL, 0);
            int retBD0 = ZAux_BusCmd_Datum(handle, FRONT_STEER, 22);
            commandCheckHandler("ZAux_BusCmd_Datum", retBD0);
            if (!retBD0)
            {
                int tmp = 0;
                while (ros::ok() && tmp < 20)
                {
                    sleep(1);
                    tmp++;
                    ZAux_Direct_GetHomeStatus(handle, FRONT_STEER, &homestatus); // 获取回零运动完成状态
                    // float tmp_position;
                    // ZAux_Direct_GetMpos(handle,FRONT_STEER,&tmp_position);
                    // int tmp_enable;
                    // ZAux_Direct_GetAxisEnable(handle,FRONT_STEER,&tmp_enable);

                    if (homestatus == 1)
                    {
                        ROS_INFO("***前转向轮回零了***\n");
                        break;
                    }
                    ROS_INFO("***前转向轮正在回零***\n");
                }
                if (tmp == 20)
                {
                    agv_msg::error_log msg;
                    msg.device_num = 0;
                    msg.error_type = 2;
                    msg.error_msg = "FRONT_STEER reset failed";
                    log_pub_.publish(msg);
                    return false;
                }
            }
            ZAux_Direct_SetAxisEnable(handle, BACK_STEER, 1);

            // 后轮转向电机回零运动
            ZAux_Direct_SetAxisEnable(handle, BACK_WHEEL, 1);
            ZAux_Direct_SetAxisEnable(handle, FRONT_STEER, 0);
            ZAux_Direct_SetAxisEnable(handle, FRONT_WHEEL, 0);

            int retBD2 = ZAux_BusCmd_Datum(handle, BACK_STEER, 22);
            commandCheckHandler("ZAux_BusCmd_Datum", retBD2);
            // 后轮转向电机回零运动
            if (!retBD2)
            {
                int tmp = 0;
                while (ros::ok() && tmp < 20)
                {
                    sleep(1);
                    tmp++;
                    ZAux_Direct_GetHomeStatus(handle, BACK_STEER, &homestatus); // 获取回零运动完成状态
                    if (homestatus == 1)
                    {
                        ROS_INFO("***后转向轮回零了***\n");
                        break;
                    }
                    ROS_INFO("***后转向轮正在回零***\n");
                }
                if (tmp == 20)
                {
                    agv_msg::error_log msg;
                    msg.device_num = 0;
                    msg.error_type = 3;
                    msg.error_msg = "BACK_STEER reset failed";
                    log_pub_.publish(msg);
                    resp.agv_result = 1;
                    return false;
                }
            }
            ZAux_Direct_SetAxisEnable(handle, FRONT_STEER, 1);
            ZAux_Direct_SetAxisEnable(handle, FRONT_WHEEL, 1);

            ZAux_Direct_SetAtype(handle, FRONT_STEER, 66);
            ZAux_Direct_SetAtype(handle, BACK_STEER, 66);
            ZAux_Direct_SetMpos(handle, FRONT_WHEEL, 0);
            sleep(0.5);
            ZAux_Direct_SetMpos(handle, BACK_WHEEL, 0);
            sleep(0.5);

            // ZAux_Direct_SetUnits(handle, FRONT_STEER, 200000 / 360);
            // ZAux_Direct_SetUnits(handle, BACK_STEER, 200000 / 360);
            // ZAux_Direct_SetUnits(handle, FRONT_WHEEL, 170000 / 360);
            // ZAux_Direct_SetUnits(handle, BACK_WHEEL, 170000 / 360);

            // 回零消息客户端
            If_zero_ = nh.serviceClient<agv_msg::hw_srv>("ifzero_srv");
            ros::service::waitForService("ifzero_srv");
            agv_msg::hw_srv iz;
            iz.request.if_zero = 1;
            If_zero_.call(iz);
            return true;
        }
        if (status_ == 2)
        { // 夹爪回零
            resp.grab_result = 0;
            ZAux_Direct_Single_Vmove(handle, GRAB_OPEN, -1);
            int tmp = 0;
            while (ros::ok() && tmp < 20) // 等待轴 4 回零运动完成
            {
                sleep(1);
                tmp++;
                ZAux_Direct_GetAxisStatus(handle, GRAB_OPEN, &axisstatus); // 获取回零运动完成状态
                ROS_INFO("axis status: %d", axisstatus);
                if (axisstatus == 32 && tmp < 20)
                {
                    ZAux_Direct_SetMpos(handle, GRAB_OPEN, 0);
                    ROS_INFO("***夹爪开合电机回零了***\n");
                    break;
                }
                if (tmp == 20)
                {
                    agv_msg::error_log msg;
                    msg.device_num = 0;
                    msg.error_type = 4;
                    msg.error_msg = "grab reset failed";
                    log_pub_.publish(msg);
                    resp.grab_result = 0;
                    return false;
                }
            }
            sleep(0.5);

            ZAux_Direct_Single_Move(handle, GRAB_LEFT, 15);
            ZAux_Direct_Single_Vmove(handle, GRAB_LEFT, -1);
            tmp = 0;
            while (ros::ok() && tmp < 20) // 等待轴 2 回零运动完成
            {
                sleep(1);
                tmp++;
                ZAux_Direct_GetAxisStatus(handle, GRAB_LEFT, &axisstatus); // 获取回零运动完成状态
                if (axisstatus == 32)
                {
                    ZAux_Direct_SetMpos(handle, GRAB_LEFT, 0);
                    sleep(0.5);
                    ZAux_Direct_SetMpos(handle, GRAB_RIGHT, 0);
                    sleep(0.5);
                    ROS_INFO("***夹爪升降电机回零了***\n");
                    resp.grab_result = 1;
                    break;
                }
                if (tmp == 20)
                {
                    agv_msg::error_log msg;
                    msg.device_num = 0;
                    msg.error_type = 4;
                    msg.error_msg = "grab reset failed";
                    log_pub_.publish(msg);
                    resp.grab_result = 0;
                    return false;
                }
            }
        }
        return true;
    }

    bool Agv_hw_interface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        status_ = 0;
        setlocale(LC_ALL, "");
        bool ret = robot_hw_nh.getParam("/agv_sim/robot_hw_test/joints", agv_joint_name); // 从参数服务器中获取name
        if (ret == true)
        {
            std::cout << "\033[1;32;40m 获得jointname成功\033[0m" << std::endl;
        }
        else
        {
            std::cout << "\033[1;31;40m 获得jointname失败\033[0m" << std::endl;
        }

        ROS_ERROR("getParam ret= %d", ret);

        agv_num_joints_ = agv_joint_name.size();

        ROS_ERROR("agv_joint_name.size()= %d", agv_joint_name.size());

        for (size_t i = 0; i < agv_num_joints_; i++)
        {
            ROS_ERROR("jointname=%s", agv_joint_name[i].c_str());
        }
        joint_effort_state.resize(agv_num_joints_);
        joint_position_state.resize(agv_num_joints_);
        joint_velocity_state.resize(agv_num_joints_);
        joint_velocity_command.resize(agv_num_joints_);

        for (int i = 0; i < agv_num_joints_; i++)
        {
            // State
            hardware_interface::JointStateHandle jointStateHandle(agv_joint_name[i], &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
            joint_state_interface.registerHandle(jointStateHandle);
            ROS_INFO("joint_name[%d].c_str()=%s", i, jointStateHandle.getName());

            // Velocity
            hardware_interface::JointHandle jointVelocityHandle(joint_state_interface.getHandle(agv_joint_name[i]), &joint_velocity_command[i]);
            velocity_joint_interface.registerHandle(jointVelocityHandle);
        }

        registerInterface(&joint_state_interface); // 将类中的接口注册到ros中
        registerInterface(&velocity_joint_interface);

        //****************************************************************************************************
        // PCI初始化
        uint32 cardnumcardnum = 0;
        int retPCI = ZAux_OpenPci(cardnumcardnum, &handle); // 以太网连接控制器
        commandCheckHandler("ZAux_OpenPci", retPCI);
        if (ERR_SUCCESS != retPCI)
        {
            printf("PCI 连接失败!\n");
            handle = NULL;
            agv_msg::error_log msg;
            msg.device_num = 0;
            msg.error_type = 0;
            msg.error_msg = "PCI connection failed";
            log_pub_.publish(msg);
            return false;
        }
        else
            printf("PCI 连接成功!\n");

        //*********************************************************************************************************
        int AxisEnable;
        for (int axis = 0; axis < 7; axis++)
        {
            ZAux_BusCmd_DriveClear(handle, axis, 0);                 // 0-清除当前 1-清除历史 2-清除外部输入警告
            int retSAE = ZAux_Direct_SetAxisEnable(handle, axis, 1); // 设置轴使能： 0 表示关闭， 1 表示打开
            commandCheckHandler("ZAux_Direct_SetAxisEnable", retSAE);
            if (retSAE == 0)
                ROS_INFO("轴%d打开使能\n", axis);
            else
            {
                agv_msg::error_log msg;
                msg.device_num = 0;
                msg.error_type = 1;
                msg.error_msg = "axis enable failed";
                log_pub_.publish(msg);
                return false;
            }
        }

        // 机器人参数初始化********************************************************************************************
        for (int i = 0; i < 7; i++)
        {
            if (i == FRONT_STEER || i == BACK_STEER)
            { // AGV电机参数初始化
                ZAux_Direct_SetUnits(handle, i, 200000.0 / 360.0);
                ZAux_Direct_SetAtype(handle, i, 66); // 设置各轴的类型为66（EtherCAT总线周期速度模式）
            }
            if (i == FRONT_WHEEL || i == BACK_WHEEL)
            { // AGV电机参数初始化
                ZAux_Direct_SetUnits(handle, i, 170000.0 / 360.0);
                ZAux_Direct_SetAtype(handle, i, 66); // 设置各轴的类型为66（EtherCAT总线周期速度模式）
            }
            // 夹爪参数设置
            if (i == GRAB_LEFT)
            {                                                  // 夹爪上升主动电机参数初始化
                ZAux_Direct_SetAtype(handle, i, 65);           // 设置各轴的类型为65（EtherCAT总线周期位置模式）
                ZAux_Direct_SetUnits(handle, i, 625000 / 471.0); // 设置各轴脉冲当量
                ZAux_Direct_SetSpeed(handle, i, 50);           // 设置各轴速度
                ZAux_Direct_SetAccel(handle, i, 100);          // 设置各轴加速度
                ZAux_Direct_SetDecel(handle, i, 100);          // 设置各轴减速度
                ZAux_Direct_SetFastDec(handle, i, 1000);       // 设置转向急停时快减速度
                ZAux_Direct_SetSramp(handle, i, 200);          // 设置各轴S曲线时间
                ZAux_Direct_SetCreep(handle, i, 20);           // 设置回零时反向爬行速度
                ZAux_Direct_SetFwdIn(handle, i, 160);          // 设置电机正限位开关IO
                ZAux_Direct_SetRevIn(handle, i, 144);          // 设置电机负限位开关IO
            }

            if (i == GRAB_RIGHT)
            {                                                  // 夹爪上升从动电机参数初始化
                ZAux_Direct_SetAtype(handle, i, 65);           // 设置各轴的类型为65（EtherCAT总线周期位置模式）
                ZAux_Direct_SetUnits(handle, i, 625000 / 471.0); // 设置各轴脉冲当量
                ZAux_Direct_SetFwdIn(handle, i, 160);
                ZAux_Direct_SetRevIn(handle, i, 144);
            }

            if (i == GRAB_OPEN)
            {
                int ret;                                   // 夹爪开合电机参数初始化
                ret = ZAux_Direct_SetAtype(handle, i, 65); // 设置各轴的类型为65（EtherCAT总线周期位置模式）
                commandCheckHandler("ZAux_Direct_SetAtype", ret);
                ZAux_Direct_SetUnits(handle, i, 10000.0 / 10.0); // 设置各轴脉冲当量为 666.66667
                ZAux_Direct_SetSpeed(handle, i, 25);             // 设置各轴速度
                ZAux_Direct_SetAccel(handle, i, 100);            // 设置各轴加速度
                ZAux_Direct_SetDecel(handle, i, 100);            // 设置各轴减速度
                ZAux_Direct_SetFastDec(handle, i, 1000);         // 设置转向急停时快减速度
                ZAux_Direct_SetSramp(handle, i, 200);            // 设置各轴S曲线时间
                ZAux_Direct_SetCreep(handle, i, 20);             // 设置回零时反向爬行速度
                ZAux_Direct_SetFwdIn(handle, i, 129);
                ZAux_Direct_SetRevIn(handle, i, 128);
            }
        }

        ZAux_Direct_Single_Addax(handle, GRAB_RIGHT, GRAB_LEFT); // 设置GRAB_RIGHT跟随GRAB_LEFT运动
        ZAux_Direct_SetMpos(handle, GRAB_OPEN, 0);
        ZAux_Direct_SetMpos(handle, GRAB_LEFT, 0);
        ZAux_Direct_SetMpos(handle, GRAB_RIGHT, 0);
        ZAux_Direct_SetMpos(handle, FRONT_STEER, 0);
        ZAux_Direct_SetMpos(handle, FRONT_WHEEL, 0);
        ZAux_Direct_SetMpos(handle, BACK_STEER, 0);
        ZAux_Direct_SetMpos(handle, BACK_WHEEL, 0);

        command_service_ = robot_hw_nh.advertiseService("/robot_control", &Agv_hw_interface::Grab_Server, this); // 启动夹爪运动服务器
        ROS_INFO("夹爪机器人服务器已启动！");
        return true;
    }

    void Agv_hw_interface::read(const ros::Time &time, const ros::Duration &period)
    {
        setlocale(LC_ALL, "");
        float velocity_unit;
        float position_unit;
        for (int i = FRONT_STEER; i < 7; i++)
        {
            int retGS = ZAux_Direct_GetMspeed(handle, i, &velocity_unit);  // 获取轴的速度(unit/s)
            int retGMPOS = ZAux_Direct_GetMpos(handle, i, &position_unit); // 获取反馈位置(unit)
            joint_position_state[i - 3] = ((double)(position_unit)) / 360.0 * 2.0 * 3.14159;
            commandCheckHandler("ZAux_Direct_GetMspeed", retGS);  // 判断指令是否执行成功
            commandCheckHandler("ZAux_Direct_GetMpos", retGMPOS); // 判断指令是否执行成功
        }
    }

    void Agv_hw_interface::write(const ros::Time &time, const ros::Duration &period)
    {
        setlocale(LC_ALL, "");
        float DAC;

        switch (status_)
        {
        case 0:
            // ***停止状态***
            for (int i = FRONT_STEER; i < FRONT_STEER + agv_num_joints_; i++)
            {
                int retSVS = ZAux_Direct_SetDAC(handle, i, 0);
                commandCheckHandler("ZAux_Direct_SetDAC", retSVS);
            }
            for (int i = FRONT_STEER; i < FRONT_STEER + agv_num_joints_; i++)
            {
                int retSVS = ZAux_Direct_Single_Cancel(handle, i, 2);
                commandCheckHandler("ZAux_Direct_SetDAC", retSVS);
            }
            for (int i = GRAB_OPEN; i < GRAB_RIGHT + 1; i++)
            {
                int retSS = ZAux_Direct_Single_Cancel(handle, i, 2);
                commandCheckHandler("ZAux_Direct_Single_Cancel", retSS);
            }
            break;
        case 1:
            // AGV回零
            break;
        case 2:
            // 夹爪回零
            break;
        case 3:
            // ***夹爪与AGV共同运动***
            for (int i = FRONT_STEER; i < FRONT_STEER + agv_num_joints_; i++)
            {
                if (i == FRONT_STEER || i == BACK_STEER)
                {
                    DAC = joint_velocity_command[i - 3] * 200000 / 3.14159 / 2; // 转向电机用户单位速度转换为电机转速
                    int retSD = ZAux_Direct_SetDAC(handle, i, DAC);
                    commandCheckHandler("ZAux_Direct_SetDAC", retSD);
                }
                else
                {
                    DAC = joint_velocity_command[i - 3] * 170000 / 3.14159 / 2; // 行走电机用户单位速度转换为电机转速
                    int retSD = ZAux_Direct_SetDAC(handle, i, DAC);
                    commandCheckHandler("ZAux_Direct_SetDAC", retSD);
                }
            }
            break;
        case 4:
            // ***AGV单独运动***
            for (int i = FRONT_STEER; i < FRONT_STEER + agv_num_joints_; i++)
            {
                if (i == FRONT_STEER || i == BACK_STEER)
                {
                    DAC = joint_velocity_command[i - 3] * 200000 / 3.14159 / 2; // 转向电机用户单位速度转换为电机转速 dac = 速度（rad/s）/（2*pi）*10000（pulse/r）*20（减速比）
                    int retSD = ZAux_Direct_SetDAC(handle, i, DAC);
                    commandCheckHandler("ZAux_Direct_SetDAC", retSD);
                }
                else
                {
                    DAC = joint_velocity_command[i - 3] * 170000 / 3.14159 / 2; // 行走电机用户单位速度转换为电机转速
                    int retSD = ZAux_Direct_SetDAC(handle, i, DAC);
                    commandCheckHandler("ZAux_Direct_SetDAC", retSD);
                }
            }
            break;
        case 5:
            // ***夹爪单独运动***
            ZAux_Direct_SetDAC(handle, FRONT_STEER, 0);
            ZAux_Direct_SetDAC(handle, FRONT_WHEEL, 0);
            ZAux_Direct_SetDAC(handle, BACK_STEER, 0);
            ZAux_Direct_SetDAC(handle, BACK_WHEEL, 0);
            break;
        default:
            break;
        }
    }

}
PLUGINLIB_EXPORT_CLASS(xj_control_ns::Agv_hw_interface, hardware_interface::RobotHW)