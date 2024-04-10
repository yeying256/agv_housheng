
#include "hfsm/AutoState.h"

namespace hfsm_ns
{
    void AutoState::start()
    {
        printf_yellow("AutoState start");
        // this->_context->srv.request.status = 3;
        // this->_context->srv.request.high = 0;
        // this->_context->srv.request.width = 0;
        // if (this->_context->client.call(this->_context->srv))
        // {
        //     ROS_INFO("GRAB BACK TO RIGHT");
        // }
        // else
        //     ROS_ERROR("GRAB BACK FAILED!");

        //std::function <>中的内容EventDeal是函数的返回值，EventData是传入参数，std::placeholders::_1是占位符
        std::function<EventDeal(EventData &)> func = std::bind(&AutoState::DealEvent, this, std::placeholders::_1);
        set_event_func(func);

        // //进来的时候就让自动导航可以运行
        // _context->nav_flag =0;
    }

    EventDeal AutoState::DealEvent(EventData &event_data)
    {
        if ((EventS)event_data._event_type == checkheart_)
        {
            ROS_INFO("AUTO WORKING");
        }
        if ((EventS)event_data._event_type == go_)
        {
            #ifdef USE_MOVE_BASE
            actionlib::SimpleClientGoalState state = this->_context->nav.ac.getState();
            #else

            #endif
            ROS_INFO("GO!!!");


            agv_msg::grab_agv srv;
            //agv单独运动
            srv.request.status = 4;
            if (this->_context->client.call(srv))
            {
                ROS_INFO("AGV MOVE!");
            }
            else
                ROS_INFO("AGV MOVE FAILED!");

            ROS_INFO("send goal");
            goal_data *goal = event_data.GetData<goal_data>();

            int ret = this->_context->nav.set_goal(goal->set_x, goal->set_y, goal->orientation);
            
            #ifdef USE_MOVE_BASE
            std::cout << "ac return " << state.toString() << std::endl;
            #endif


            while(ros::ok())
            {
                // sleep(0.1);
                ros::Duration(0.1).sleep();
                // std::cout<<"正在前往目标点"<<std::endl;
                // std::cout<<"this->_context->GetCurStateName() = "<<this->_context->GetCurStateName()<<std::endl;
                if (this->_context->GetCurStateName() == "Lock"||this->_context->GetCurStateName() == "Idle")
                {
                printf_yellow("lock1");

                    #ifdef USE_MOVE_BASE
                    this->_context->nav.ac.cancelGoal();
                    #else
                    agv_msg::GP_nav gp_nav;
                    gp_nav.request.state=0;
                    _context->nav.GP_srv_.call(gp_nav);
                    #endif

                    event_data._event_type = stop_;
                    break;
                }
                #ifdef USE_MOVE_BASE
                if (this->_context->nav.ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || this->_context->GetCurStateName() == "Lock")
                {
                    break;
                }
                #else
                // 查询是否结束

                agv_msg::GP_nav gp_nav;
                gp_nav.request.state = 2;
                _context->nav.GP_srv_.call(gp_nav);
                
                if (gp_nav.response.success == true)
                {   
                    // 如果查询结果成功了，那么就跳出循环
                    break;
                }
                #endif
            }
            // this->_context->nav_flag = 1;
        }
        //放料
        if ((EventS)event_data._event_type == put_)
        {
            _context->nav_flag = 1;
            geometry_msgs::Pose test1;
            std::vector<geometry_msgs::Pose>* way_point = event_data.GetData<std::vector<geometry_msgs::Pose>>();

            printf_yellow("put_ event test");
            printf("way_point->size() = %d \n",way_point->size());
            printf_yellow("put_ event test");

            

            
            for (int i = 0; i < way_point->size(); i++)
            {
                way_point->at(i);
                //发送目标点
                int ret = this->_context->nav.set_goal(way_point->at(i).position.x, way_point->at(i).position.y, way_point->at(i).orientation);
                printf_yellow("put_ event test2");
                
                while(ros::ok())
                {
                    sleep(0.1);
                    // std::cout<<"正在前往目标点"<<std::endl;
                    // std::cout<<this->_context->GetCurStateName()<<std::endl;

                    
                    if (this->_context->GetCurStateName() == "Lock"||this->_context->GetCurStateName() == "Idle")
                    {
                    printf_yellow("lock1");

                        #ifdef USE_MOVE_BASE
                        this->_context->nav.ac.cancelGoal();
                        #else
                        agv_msg::GP_nav gp_nav;
                        gp_nav.request.state=0;
                        _context->nav.GP_srv_.call(gp_nav);
                        #endif

                        event_data._event_type = stop_;
                        break;
                    }
                    #ifdef USE_MOVE_BASE
                    if (this->_context->nav.ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || this->_context->GetCurStateName() == "Lock")
                    {
                        break;
                    }
                    

                    #else
                    // 查询是否结束

                    agv_msg::GP_nav gp_nav;
                    gp_nav.request.state = 2;
                    _context->nav.GP_srv_.call(gp_nav);
                    
                    if (gp_nav.response.success == true)
                    {   
                        // 如果查询结果成功了，那么就跳出循环
                        break;
                    }
                    #endif

                }
                //当 没到目标点并且 不是锁定状态时 跳出循环。

                
                //当状态等于锁定的时候，停止运动并跳出for循环
                if (this->_context->GetCurStateName() == "Lock"||this->_context->GetCurStateName() == "Idle")
                {
                    printf_yellow("lock2");
                    #ifdef USE_MOVE_BASE
                    this->_context->nav.ac.cancelGoal();
                    this->_context->nav.ac.waitForResult();
                    #else
                    agv_msg::GP_nav gp_nav;
                    gp_nav.request.state=0;
                    _context->nav.GP_srv_.call(gp_nav);
                    #endif

                    event_data._event_type = stop_;

                    #ifdef USE_MOVE_BASE
                    if (this->_context->nav.ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
                    {
                        ROS_INFO("You have canceled the goal!");
                    }
                    #else
                    // 查询状态是否还在运行中
                    gp_nav.request.state = 2;
                    _context->nav.GP_srv_.call(gp_nav);
                    if (gp_nav.response.success == true)
                    {
                        // 如果状态已经success了，说明可以继续发布指令了，说明急停成功
                        ROS_INFO("You have canceled the goal!");
                    }
                    #endif
                    else
                    {
                        ROS_ERROR("The cancel goal failed for some reason");
                    }

                    break;
                }
                
            }
            
            //取完或者中途停止清零状态标志位 
            // _context->nav_flag = 0;
        }


        if ((EventS)event_data._event_type == automove_grabmove_)
        {
            struct grab_move* grab_move = event_data.GetData<struct grab_move>();
            this->grab_move(grab_move->width,grab_move->heigh,grab_move->if_width);
        }


        if ((EventS)event_data._event_type == stop_)
        {
            ROS_INFO("stop navigation!");
            #ifdef USE_MOVE_BASE
            this->_context->nav.ac.cancelGoal();
            actionlib::SimpleClientGoalState state = this->_context->nav.ac.getState();
            std::cout << "current nav state is : " << state.toString() << std::endl;
            // Wait for the action to return
            this->_context->nav.ac.waitForResult();
            if (this->_context->nav.ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
            {
                ROS_INFO("you have canceled the goal!");
            }
            else
            {
                ROS_ERROR("cancel_goal failed");
            }
            #else

            agv_msg::GP_nav gp_nav;

            // 生成急停命令
            gp_nav.request.state=0;
            _context->nav.GP_srv_.call(gp_nav);

            // 查询状态
            gp_nav.request.state=2;
            _context->nav.GP_srv_.call(gp_nav);

            while (ros::ok())
            {
                // 检查是否真的成功了
                if (gp_nav.response.success == true)
                {
                    ROS_INFO("you have canceled the goal!");
                    break;
                }else{
                    sleep(0.1);
                    // 再发一个指令
                    _context->nav.GP_srv_.call(gp_nav);
                    ROS_INFO_STREAM(gp_nav.response.msg);
                }
                
            }
            

            #endif
        }
        
        return tail;
    }

    void AutoState::stop()
    {
        printf_green("AutoState stop");
    }

    void AutoState::update()
    {
        TransState("Adjust");
    }


    /**
     * @brief 夹爪运动
     * 
     * @param width 
     * @param hight 
     * @param width_move 
     * @return true 
     * @return false 
     */
    bool AutoState::grab_move(double width,double hight,bool width_move)
    {
        agv_msg::grab_agv srv;
        // 状态夹爪单独运动
        srv.request.status = 5;
        //储存宽度信息
        if (width_move == true)//如果宽度能动
        {
            printf_yellow("指令");
            std::cout<<"夹爪指令宽度 = "<<width<<std::endl;
            srv.request.width = AdjustState::RealData2SrvData(width,AdjustState::Data_type::WIDTH);
        }else//否则，宽度不要动
        {
            srv.request.width = 999;//告诉他不要动
        }

        // 计算高度信息
        std::cout<<"夹爪指令高度 = "<<hight<<std::endl;


        srv.request.high = AdjustState::RealData2SrvData(hight,AdjustState::Data_type::HEIGHT);
        printf_yellow("指令结束");

        if (!this->_context->client.call(srv))
        {
            return false;
        }
        return true;
    }
}
