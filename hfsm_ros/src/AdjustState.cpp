#include "hfsm/AdjustState.h"

namespace hfsm_ns
{
    AdjustState::AdjustState(ros::NodeHandle nh) : n(nh)
    {
        // 视觉相对运动节点
        visual_client = n.serviceClient<agv_msg::visual_point_move>("/visual_point_move");
    };

    void AdjustState::start()
    {
        printf_yellow("Adjust start");
        workbench_map = retUmap();
        this->_context->detector_type = feedback_;
        std::function<EventDeal(EventData &)> func = std::bind(&AdjustState::DealEvent, this, std::placeholders::_1);
        set_event_func(func);
    }

    EventDeal AdjustState::DealEvent(EventData &event_data)
    {
        if ((EventS)event_data._event_type == task_)
        {
            ar_data *task_msg = event_data.GetData<ar_data>();
            std::cout << "task_msg.height" << task_msg->pre_height << std::endl;
            this->run(task_msg->pre_height, task_msg->pre_width, task_msg->width, task_msg->height, task_msg->workbench_id);
            return tail;
        }

        if ((EventS)event_data._event_type == photo_)
            return keep_on;
    }

    void AdjustState::stop()
    {
        printf_green("Adjust stop");
    }

    double AdjustState::RealData2SrvData(double RealData, Data_type Types)
    {
        const double topBoard2init = 208.0;   // 机器人上顶板到夹爪零位的高度
        const double ground2topBoard = 380.0; // 地面到agv上顶板的高度
        const double grab_thick = 100.0;      // 夹爪末端的相对高度

        const double width_init = 580.0; // 夹爪零位的宽度

        double data; // 计算出来的数据

        switch (Types)
        {
        case WIDTH:
            // 因为是双向的丝杆，所以减去基础长度之后要除以2
            data = (RealData - width_init) / 2.0;
            break;
        case HEIGHT:
        {
            data = RealData - topBoard2init - ground2topBoard - grab_thick;
            break;
        }

        default:
            break;
        }

        // 将数据返回
        return data;
    }

    bool AdjustState::grab_move(double width, double hight, bool width_move)
    {
        agv_msg::grab_agv srv;
        // 状态夹爪单独运动
        srv.request.status = 5;
        // 储存宽度信息
        if (width_move == true) // 如果宽度能动
        {
            printf_yellow("指令");
            std::cout << "夹爪指令宽度 = " << width << std::endl;
            srv.request.width = RealData2SrvData(width, WIDTH);
        }
        else // 否则，宽度不要动
        {
            srv.request.width = 999; // 告诉他不要动
        }

        // 计算高度信息
        std::cout << "夹爪指令高度 = " << hight << std::endl;

        srv.request.high = RealData2SrvData(hight, HEIGHT);
        printf_yellow("指令结束");

        if (!this->_context->client.call(srv))
        {
            return false;
        }
        return true;
    }

    bool AdjustState::Switch_agv_mode(int mode)
    {
        agv_msg::grab_agv srv;
        srv.request.status = mode;
        switch (mode)
        {
        case 0:
            if (this->_context->client.call(srv))
            {
                ROS_INFO("SWITCH to STOP mode");
            }
            else
            {
                ROS_ERROR("FAILED switch to STOP mode");
                return false;
            }

            break;
        case 1:
            if (this->_context->client.call(srv))
            {
                ROS_INFO("SWITCH to !back to zero! mode");
            }
            else
            {
                ROS_ERROR("FAILED switch to !back to zero! mode");
            }
            break;

        case 2:
            if (this->_context->client.call(srv))
            {
                ROS_INFO("SWITCH to !grab to zero! mode");
            }
            else
            {
                ROS_ERROR("FAILED switch to !grab to zero! mode");
            }
            break;

        case 3:
        case 5:
            ROS_ERROR("You can not use this function: Switch_agv_mode()");
            break;
        case 4:
            if (this->_context->client.call(srv))
            {
                ROS_INFO("SWITCH to AGV_only mode");
            }
            else
            {
                ROS_ERROR("FAILED switch to AGV_only mode");
            }
            break;
        default:
            break;
        }
        return true;
    }

    /**
     * @brief 相对运动指令
     *
     * @param x_d 期望的x
     * @param y_d 期望的y
     * @param yaw_d 期望的yaw
     * @param feedback 反馈的pos2d
     * @return true
     * @return false
     */
    bool AdjustState::relative_move_cmd(double x_d, double y_d, double yaw_d, geometry_msgs::Pose2D feedback)
    {
        // 切换到agv单独运动状态
        if (Switch_agv_mode(4))
        {
            ROS_INFO("AGV MOVE!");
        }
        else
            ROS_ERROR("AGV MOVE FAILED!");

        req.feed_back_pose = feedback;
        req.target_pose.x = x_d;
        req.target_pose.y = y_d;
        req.target_pose.theta = yaw_d;
        req.state = 1;

        if (visual_client.call(req, resp))
        {
            ;
        }
        else
        {
            std::cout << "/visual_point_move服务调用失败" << std::endl;
            return false;
        }

        // 打印误差
        std::cout << req << std::endl;
        std::cout << resp.msg << std::endl;
        std::cout << req << std::endl;
        std::cout << resp.msg << std::endl;
        // 一直查询是否结束运动
        do
        {
            if (this->_context->GetCurStateName() == "Lock")
                break; // 跳出循环
            req.state = 2;
            // 相对运动查询状态：1是运行，0是停止 2是查询状态
            bool ok = visual_client.call(req, resp);
            if (ok == true)
            {
                ROS_INFO("visual target server: [%s]", resp.msg.c_str());
            }
            else
            {
                ROS_ERROR("failed to call visual_target service");
            }
            sleep(1);
        } while (resp.success == false && this->_context->GetCurStateName() == "Adjust");
        // 当没运动结束并且是调整状态的时候，持续执行循环

        // 如果进入了锁定状态，那么久发送0停止
        if (this->_context->GetCurStateName() == "Lock")
        {
            // req.state = 0;
            // //发送0，停止
            // visual_client.call(req, resp);
            relative_move_cmd(0);
            if (resp.success == true)
            {
                ROS_INFO("visual target server: [%s]", resp.msg.c_str());
            }
            else
            {
                ROS_ERROR("failed to call visual_target service");
            }
            return false;
        }

        return true;
    }

    bool AdjustState::relative_move_cmd(int mode)
    {
        agv_msg::visual_point_move::Request req1;

        switch (mode)
        {
        case 0:
        case 2:
            req1.state = mode;

            break;

        default:
            std::cout << "你不能用这个方式调用这个函数,模式不是0或者2就不能调用。" << std::endl;
            return false;
            break;
        }

        if (visual_client.call(req1, resp))
        {
            ;
        }
        else
        {
            std::cout << "/visual_point_move服务调用失败" << std::endl;
            return false;
        }
        return true;
        ;
    }

    void AdjustState::run(int pre_height, int pre_width, int grab_width, int grab_height, int workbench_id)
    {
        ROS_INFO("GO GET OBJECT NO.[%d] ...", workbench_id);

        // 等待夹爪运动的服务开启
        this->_context->client.waitForExistence();

        // 夹爪运动的数据 5夹爪单独运动
        // agv_msg::grab_agv srv;
        // 0:急停；1：AGV回零；2：夹爪回零；3：两部分一起运动；4：AGV单独运动；5：夹爪单独运动
        // srv.request.status = 5;
        // srv.request.width = 999;//告诉他不要动
        // srv.request.high = pre_height;

        // 夹爪高度到预备高度，宽度不动
        if (grab_move(999, pre_height, false))
        {
            ROS_INFO("GRAB MOVE!");
        }
        else
        {
            ROS_ERROR("GRAB LIFT FAILED!");
        }

        //     x: 1.1960333585739136
        // y: 0.08133205771446228
        // theta: 0.023117946460843086

        // 进入预备点
        if (!relative_move_cmd(1.196033, 0.081332, 0.0231179, _context->object_list[0].pose))
        {
            return;
        }

        // 预备向里面伸进去，将宽度改变为料的宽度
        double target_length = _context->object_list[0].length + 85.0;
        if (grab_move(target_length, pre_height, true))
        {
            ROS_INFO("GRAB MOVE!");
        }
        else
        {
            ROS_ERROR("GRAB LIFT FAILED!");
        }

        // 调整一下
        if (!relative_move_cmd(1.196033, 0.081332, 0.0231179, _context->object_list[0].pose))
        {
            return;
        }
        // 再调整一下
        if (!relative_move_cmd(1.196033, 0.081332, 0.0231179, _context->object_list[0].pose))
        {
            return;
        }

        // 重新调整夹爪宽度
        if (grab_move(target_length, pre_height, true))
        {
            ROS_INFO("GRAB MOVE!");
        }
        else
        {
            ROS_ERROR("GRAB LIFT FAILED!");
        }

        // x: 0.7971910238265991
        //     y: 0.04687689617276192
        //     theta: 0.006656110752373934
        // 进去
        if (!relative_move_cmd(0.797191, 0.04687689, 0.0066561, _context->object_list[0].pose))
        {
            return;
        }

        sleep(2.0);

        // 抬升
        if (grab_move(999, grab_height, false))
        {
            ROS_INFO("GRAB MOVE!");
        }
        else
            ROS_INFO("GRAB LIFT FAILED!");

        //         x: 1.3383946418762207
        // y: 0.026927808299660683
        // theta: 0.012390093877911568

        // 进去
        //      x: 0.8226519227027893
        //  y: 0.02070448361337185
        //  theta: 2.9485563572961837e-05

        // agv进去
        //  if(!relative_move_cmd(0.854,0.11,0.045,_context->object_list[0].pose))
        //  {
        //  return;
        //  }

        // 改变为抬起来的
        // if (grab_move(target_length,grab_height,true))

        this->_context->take_flag = 1;

        geometry_msgs::Pose2D back_tmp;
        back_tmp.x = 0.0;
        back_tmp.y = 0.0;
        back_tmp.theta = 0.0;
        // 撤走的代码
        if (!relative_move_cmd(0.5, 0.0, 0.0, back_tmp))
        {
            return;
        }

        // 夹爪下降
        if (grab_move(999, pre_height, false))
        {
            ROS_INFO("GRAB MOVE!");
        }
        else
            ROS_INFO("GRAB LIFT FAILED!");

        if (Switch_agv_mode(4))
        {
            ROS_INFO("AGV MOVE!");
        }
        else
            ROS_INFO("AGV MOVE FAILED!");

        // 放料

        // srv.request.status = 4;
        // 切换到agv单独运动模式

        // 跟相对运动通讯
        // 1是运行，0是停止 2是查询状态
        // req.state = 1;
        // req.target_pose.x = 1.0;
        // req.target_pose.y = -0.2;
        // req.target_pose.theta = 0.0;
        // // memcpy(&req.err, &find_ar.p[i].err, 3 * sizeof(float));
        // // 将第0个目标的信息传进来
        // req.feed_back_pose = this->_context->object_list[0].pose;
        // visual_client.call(req, resp);

        // 预备状态

        /*
                // 打印误差
                std::cout << req << std::endl;
                std::cout << resp.msg << std::endl;
                // 一直查询是否结束运动
                do
                {
                    if (this->_context->GetCurStateName() == "Lock")
                        break;//跳出循环
                    req.state = 2;
                    // 相对运动查询状态：1是运行，0是停止 2是查询状态
                    bool ok = visual_client.call(req, resp);
                    if (ok == true)
                    {
                        ROS_INFO("visual target server: [%s]", resp.msg.c_str());
                    }
                    else
                    {
                        ROS_ERROR("failed to call visual_target service");
                    }
                    sleep(1);
                } while (resp.success == false && this->_context->GetCurStateName() == "Adjust");
                // 当没运动结束并且是调整状态的时候，持续执行循环
        */

        // 相对运动停止
        /*
        if (this->_context->GetCurStateName() == "Lock")
        {
            // req.state = 0;
            // //发送0，停止
            // visual_client.call(req, resp);
            relative_move_cmd(0);
            if (resp.success == true)
            {
                ROS_INFO("visual target server: [%s]", resp.msg.c_str());
            }
            else
            {
                ROS_ERROR("failed to call visual_target service");
            }
            return;
        }

        */

        // if (this->_context->take_flag != 1)
        // {

        // }
        // else
        // {
        //     srv.request.width = 999;
        //     srv.request.high = grab_height;
        //     if (this->_context->client.call(srv))
        //     {
        //         ROS_INFO("GRAB MOVE!");
        //     }
        //     else
        //         ROS_INFO("GRAB LIFT FAILED!");

        //     srv.request.width = grab_width;
        //     srv.request.high = 999;
        //     if (this->_context->client.call(srv))
        //     {
        //         ROS_INFO("GRAB MOVE!");
        //     }
        //     else
        //         ROS_INFO("GRAB LIFT FAILED!");
        // }

        // 切换到底盘单独运动

        // req.state = 1;
        // req.pose.x = 1.0;
        // req.pose.y = 0.0;
        // req.pose.theta = 0.0;
        // std::cout << req.err[0] << ", " << req.err[1] << ", " << req.err[2] << std::endl;
        // visual_client.call(req, resp);
        // std::cout << resp.msg << std::endl;
        // do
        // {
        //     if (this->_context->GetCurStateName() == "Lock")
        //         break;
        //     req.state = 2;
        //     bool ok = visual_client.call(req, resp);
        //     if (resp.success == true)
        //     {
        //         ROS_INFO("visual target server: [s%]", resp.msg.c_str());
        //     }
        //     else
        //     {
        //         ROS_ERROR("failed to call visual_target service");
        //     }
        //     sleep(1);
        // } while (resp.success == false && this->_context->GetCurStateName() == "Adjust");

        // if (this->_context->GetCurStateName() == "Lock")
        // {
        //     req.state = 0;
        //     visual_client.call(req, resp);
        //     if (resp.success == true)
        //     {
        //         ROS_INFO("visual target server: [s%]", resp.msg.c_str());
        //     }
        //     else
        //     {
        //         ROS_ERROR("failed to call visual_target service");
        //     }
        //     return;
        // }

        // srv.request.status = 5;
        // srv.request.width = 999;
        // srv.request.high = 0;
        // if (this->_context->client.call(srv))
        // {
        //     ROS_INFO("GRAB MOVE!");
        // }
        // else
        //     ROS_INFO("GRAB LIFT FAILED!");

        // 已经取完料了
        this->_context->take_flag = 1;

        TransState("Auto");
    }

    void AdjustState::update()
    {
        std::cout << "manipulating ..." << std::endl;
    }
}