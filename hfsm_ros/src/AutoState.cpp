
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
        //进来的时候就让自动导航可以运行
        _context->nav_flag =0;
    }

    EventDeal AutoState::DealEvent(EventData &event_data)
    {
        if ((EventS)event_data._event_type == checkheart_)
        {
            ROS_INFO("AUTO WORKING");
        }
        if ((EventS)event_data._event_type == go_)
        {
            actionlib::SimpleClientGoalState state = this->_context->nav.ac.getState();

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
            this->_context->nav_flag = 1;

            std::cout << "ac return " << state.toString() << std::endl;
        }
        //放料
        if ((EventS)event_data._event_type == put_)
        {
            _context->nav_flag = 1;
            std::vector<geometry_msgs::Pose>* way_point = event_data.GetData<std::vector<geometry_msgs::Pose>>();
            
            for (int i = 0; i < way_point->size(); i++)
            {
                way_point->at(i);
                //发送目标点
                int ret = this->_context->nav.set_goal(way_point->at(i).position.x, way_point->at(i).position.y, way_point->at(i).orientation);
                
                do
                {
                    sleep(0.1);
                    std::cout<<"正在前往目标点"<<std::endl;
                    
                    if (this->_context->GetCurStateName() == "Lock")
                    {
                        this->_context->nav.ac.cancelGoal();
                        event_data._event_type = stop_;
                        break;
                    }
                } while (this->_context->nav.ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && this->_context->GetCurStateName() != "Lock");
                //当 没到目标点并且 不是锁定状态时 跳出循环。

                
                //当状态等于锁定的时候，停止运动并跳出for循环
                if (this->_context->GetCurStateName() == "Lock")
                {
                    this->_context->nav.ac.cancelGoal();
                    event_data._event_type = stop_;

                    this->_context->nav.ac.waitForResult();
                    if (this->_context->nav.ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
                    {
                        ROS_INFO("You have canceled the goal!");
                    }
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
