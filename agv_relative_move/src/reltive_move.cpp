#include "reltive_move.h"




namespace xj_control_ns
{

    /**
     * @brief 构造函数
     * 
     */
    reltive_move::reltive_move(/* args */)
    {
        have_target_ = false;
        //有参构造
        pid_ = new xj_dy_ns::PID_controller(20,0,0);
        //接收ar_pose
        arpos_sub_  = n.subscribe("/ar_pose_marker", 10, &reltive_move::pose_callback,this);

        visualPos_sub_ = n.subscribe("/visual_pose_fb", 10, &reltive_move::visual_callback,this);

        //接收target
        // ar_target_Server_ = n.advertiseService("/ar_track_target", &reltive_move::TrackCallback,this);

        ar_target_Server2_ = n.advertiseService("/ar_track_target2", &reltive_move::TrackCallback2,this);
        ar_target_get_ = n.advertiseService("/ar_target_get", &reltive_move::get_targetpose,this);

        visual_target_Server_ = n.advertiseService("/visual_track_target",&reltive_move::TrackCallback_visual,this);

        //发送机器人运动速度命令
        pub_cmd_vel_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    /**
     * @brief 
     * 
     * @param mode 0 点动模式
     * @return true 
     * @return false 
     */
    bool reltive_move::init(uint8_t mode)
    {
        this->Visual_mode =POINT_MOVE;
        switch (mode)
        {
        case POINT_MOVE : 
            {
                this->visualPos_point_Server_ = n.advertiseService("/visual_point_move",&reltive_move::TrackCallback_pointmove_visual,this);
                // 初始化pid参数
                point_move_struct_.pid->init(0.9,0.0,0.0,0.05);
                (point_move_struct_.pid+1)->init(0.9,0.0,0.0,0.05);
                (point_move_struct_.pid+2)->init(1.2,0.0,0.0,0.05);
                point_move_struct_.max_err[0] = 0.003;
                point_move_struct_.max_err[1] = 0.003;
                point_move_struct_.max_err[2] = 0.08;
            break;
            }
        default:
            break;
        };
    }

    /**
     * @brief Destroy the reltive move::reltive move object
     * 
     */
    reltive_move::~reltive_move()
    {
        delete pid_;
    }




    /**
     * @brief 转换ar码的姿态
     * 
     * @param ar_reltive 
     * @return geometry_msgs::Pose 
     */
    geometry_msgs::Pose reltive_move::trans_arpose(const geometry_msgs::Pose&  ar_reltive)
    {
        ar_reltive.orientation.w;
        //初始化时w在前面（w,x,y,z），但是输出和内部存储的时候w在后面(x,y,z,w)
        Eigen::Quaterniond arpose_now(ar_reltive.orientation.w,ar_reltive.orientation.x,ar_reltive.orientation.y,ar_reltive.orientation.z);
        Eigen::Matrix3d R_init = arpose_now.toRotationMatrix();//转换成旋转矩阵
        Eigen::Matrix3d R_trans;
        //ar码右侧为x，垂直纸面向外为z，向上为y
        //车体坐标系垂直纸面向里为x，向左为y，向上为z
        R_trans<<
        0,-1,0,
        0,0,1,
        -1,0,0;

        //转换完的坐标系为
        Eigen::Matrix3d R_base2ar_trans = R_init*R_trans;
        //转化为四元数
        Eigen::Quaterniond arpose_new(R_base2ar_trans);//对姿态统一坐标系
        geometry_msgs::Pose newpose;

        //位移是一样的
        newpose.position = ar_reltive.position;
        newpose.orientation.w = arpose_new.w();
        newpose.orientation.x = arpose_new.x();
        newpose.orientation.y = arpose_new.y();
        newpose.orientation.z = arpose_new.z();


        //返回坐标变换
        return newpose;
    }

    /**
     * @brief 接收到信息了如何处理 讲ar码相对坐标转化为相对坐标和欧拉角，在此项目，选用了xyz和绕z轴的旋转yaw和绕x轴的旋转roll
     * 
     * @param marks_msgs ar码原始参数(是一个向量，识别到多少个ar码就有多少个元素)
     */
    void reltive_move::pose_callback(const ar_track_alvar_msgs::AlvarMarkers& marks_msgs)
    {
        // std::cout<<"ar_reltive_.confidence = "<<marks_msgs.markers[0].confidence<<std::endl;

        if (this->move_flag_ == true)//如果是让运动，才开始下列的操作
        {
            int tmp=0;
            int tmp2=0;
            bool ar_left_resive = false;
            bool ar_right_resive = false;

            for (int i = 0; i < marks_msgs.markers.size(); i++)//遍历所有识别到的ar码
            {
                if (marks_msgs.markers[i].id==this->target_ar_id_)//如果是需要跟踪的id,那么把信息加载下来
                {
                    this->confidence_ =  marks_msgs.markers[i].confidence;
                    ar_reltive_ = this->trans_arpose(marks_msgs.markers[i].pose.pose);//将处理好的pose加载到内部参数
                    
                    // std::cout<<"\033[1;32;40mar_reltive_.orientation = "<<ar_reltive_.orientation<<"\033[0m"<<std::endl;
                    // std::cout<<"ar_reltive_.position = "<<ar_reltive_.position<<std::endl;


                    tf::Quaternion quat;
                    tf::quaternionMsgToTF(ar_reltive_.orientation, quat); //将相对方向转换成tf
                    tf::Matrix3x3(quat).getRPY(reltive_pose.roll, reltive_pose.pitch, reltive_pose.yaw);
                    tmp++;
                }

                if (marks_msgs.markers[i].id==this->left_ar_.id)
                {
                    tmp2++;
                    left_ar_.x = marks_msgs.markers[i].pose.pose.position.x;
                    left_ar_.y = marks_msgs.markers[i].pose.pose.position.y;
                    left_ar_.confidance = marks_msgs.markers[i].confidence;
                    ar_left_resive =true; //因为如果没有的话进不来只能判断是否有数据，不能判断是否丢失
                }
                if (marks_msgs.markers[i].id==this->right_ar_.id)
                {
                    tmp2++;
                    right_ar_.x = marks_msgs.markers[i].pose.pose.position.x;
                    right_ar_.y = marks_msgs.markers[i].pose.pose.position.y;
                    right_ar_.confidance = marks_msgs.markers[i].confidence;
                    ar_right_resive =true; //因为如果没有的话进不来只能判断是否有数据，不能判断是否丢失
                }
                
            }

            // 判断左右是否都收到了
            left_ar_.if_resive = ar_left_resive;
            right_ar_.if_resive = ar_right_resive;

            // std::cout<<"这里是ar码接受线程"<<"left_ar_.if_resive="<<left_ar_.if_resive<<",,right_ar_.if_resive = "<<right_ar_.if_resive<<std::endl;

            if (tmp>0)
            {this->have_target_ = true;}else{have_target_ = false;}
            
        }
    }

    /**
     * @brief 视觉的返回值
     * 
     * @param pose_msgs 
     */
    void reltive_move::visual_callback(const agv_msg::Visual_fb& pose_msgs)
    {
        // 返回像素深度 
        this->visual_pose_now_.x =  pose_msgs.x;
        // 返回 
        this->visual_pose_now_.y =  pose_msgs.y;
        // 左减去右 
        this->visual_pose_now_.theta =  pose_msgs.yaw_err;
        this->visual_have_target_ = pose_msgs.have_target;
    }



    /**
     * @brief 接收相对目标 并且让底盘相对运动到合适的位置。
     * 
     * @param req 
     * @param respon 
     * @return true 
     * @return false 
     */
    bool reltive_move::TrackCallback(agv_msg::reltive_pose::Request &req, 
                            agv_msg::reltive_pose::Response &respon)
    {
        geometry_msgs::Pose2D target_pose = req.pose;//相对机器人底盘的期望运动偏置
        xj_dy_ns::PID_controller pid_tmp[3];

        // 跟踪pid初始化
        pid_tmp[0].init(0.3,0.003,0);
        pid_tmp[1].init(0.8,0.003,0);
        pid_tmp[2].init(1.0,0.001,0);

        std::cout<<"进入了回调函数"<<std::endl;



        geometry_msgs::Twist cmd_vel;   //机器人的指令速度
        double hight = req.high;//机械手臂的宽和高
        double width = req.width;
        this->target_ar_id_ = req.target_ar_id;
        move_flag_ = true;//可以开始运动了


            //监听tf
        std::stringstream ss;
        std::string stra="";
        ss<<"/ar_marker_";
        ss<<req.target_ar_id;
        ss>>stra;
        const std::string &strb=stra;
        tf::StampedTransform transform;
        try
        {
            listener_.waitForTransform("/base_footprint",strb, ros::Time(0), ros::Duration(3.0));
            listener_.lookupTransform("/base_footprint",strb, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("TF may have some problems.\n %s",ex.what());
            ros::Duration(1.0).sleep();
            respon.msg = "success";
            respon.success = false;
            return true;
        }




        double err[3];


        double err_all = 0;
        double cmd_all = 0;


        

        err[0] = target_pose.x - this->ar_reltive_.position.x;
        err[1] = target_pose.y - this->ar_reltive_.position.y;
        err[2] = target_pose.theta - reltive_pose.yaw;



        for (int i = 0; i < 2; i++)
        {
            err_all+=abs(err[i]);
        }//计算总体误差
        
        
        //  运动的代码
        ros::Duration time;
        time.fromSec(0.1);
        for (int i = 0; i < 3; i++)
        {
            std::cout<<"err"<<i<<"="<<err[i]<<std::endl;
        }
        sleep(0.5);
        int tmp = 100;
        while ((ros::ok())&&((err_all>0.005) || (abs(err[2])>0.05)) )
        {
            
            
            cmd_vel.linear.x = pid_tmp[0].PID(-err[0]);
            //测试
            cmd_vel.linear.y = pid_tmp[1].PID(-err[1]);
            cmd_vel.angular.z = pid_tmp[2].PID(-err[2]);
            // std::cout<<"cmd_vel="<<cmd_vel<<std::endl;
            pub_cmd_vel_.publish(cmd_vel);
            
            time.sleep();
            err[0] = target_pose.x - this->ar_reltive_.position.x;
            err[1] = target_pose.y - this->ar_reltive_.position.y;
            err[2] = target_pose.theta - reltive_pose.yaw;

            err_all = abs(err[0])+abs(err[1]);

            // for (int i = 0; i < 3; i++)
            // {
            //     std::cout<<"err"<<i<<"="<<err[i]<<std::endl;
            // }
            // std::cout<<"reltive_pose.yaw = "<<reltive_pose.yaw<<std::endl;

            //检查是否还有这个坐标系
            if (have_target_ == false)
            {
                // 如果不存在ar码的坐标系了，那就跳出循环，并返回跳出边界
                respon.msg = "Leaving the boundary";
                respon.success = false;
                break;
            }

        }

        //停止
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z= 0.0;

        pub_cmd_vel_.publish(cmd_vel);

        //
        usleep(500*1000);
        
        //  运动的代码

        move_flag_=false;//将标志位设置为 否，接收到数据不再读取

        if (respon.success != false)
        {
            respon.success = true;//给客户端返回成功
            respon.msg = "reltive move success";
        }
        return true;
    }


    /**
     * @brief 
     * 
     * @param req 
     * @param respon 
     * @return true 
     * @return false 
     */
    bool reltive_move::TrackCallback2(agv_msg::reltive_pose_ar::Request &req, 
                            agv_msg::reltive_pose_ar::Response &respon)
    {
        // geometry_msgs::Pose2D target_pose = req.pose;//相对机器人底盘的期望运动偏置

        // 0是停止
        switch (req.state)
        {
        case 0: //停止
            move_flag_=false;
            feedback_state_ = INTE_STOP;//中断停止
            respon.success = true;
            respon.msg="reltive move: I get the stop cmd ,stoping...";
        break;
        
        case 1: //指令状态
            
        target_pose_ = req.pose;//相对机器人底盘的期望运动偏置
        this->left_ar_.id = req.left_ar_id;
        this->right_ar_.id = req.right_ar_id;
        std::cout<<"left_ar_.id = "<<left_ar_.id<<std::endl;
        std::cout<<"right_ar_.id = "<<right_ar_.id<<std::endl;
        std::cout<<"ar_err reciveerr0 = "<<req.err[0]<<std::endl;
        std::cout<<"ar_err reciveerr1 = "<<req.err[1]<<std::endl;
        std::cout<<"ar_err reciveerr2 = "<<req.err[2]<<std::endl;

        left_ar_.if_resive = false;
        right_ar_.if_resive = false;
        move_flag_ = true;//可以开始运动了
            for (int i = 0; i < 3; i++)
            {
                err_min_[i] = req.err[i];
            }
        feedback_state_ = RUN;
            respon.success = true;
            respon.msg="reltive move: I get the target cmd ,runing...";
        break;

        case 2: //查询状态
        switch (feedback_state_)
        {
        case WAIT:
            respon.success = true;
            respon.msg = "reltive move:Waiting target";
            break;
        case RUN:
            respon.success = false;
            respon.msg = "reltive move: Runing, wait some time";
            break;
        case INTE_STOP:
            respon.success = false;
            respon.msg = "reltive move: interupting ...";
            break;
        case COMPLATE:
            respon.success = true;
            respon.msg = "reltive move: acomplish move wait next target";
            break;
        case OUT:
            respon.success = true;
            respon.msg = "reltive move: lost ar target maybe ar is lost, you can resend the target ";
            break;
        default:
            break;
        }

        
        }
        
        return true;
        
    }


    /**
     * @brief 回调函数
     * 
     * @param req 
     * @param respon 
     * @return true 
     * @return false 
     */
    bool reltive_move::TrackCallback_visual(agv_msg::reltive_pose_visual::Request &req, 
                            agv_msg::reltive_pose_visual::Response &respon)
    {
        // 0是停止
        switch (req.state)
        {
        case 0: //停止
            move_flag_=false;
            visual_state_ = INTE_STOP;//中断停止
            respon.success = true;
            respon.msg="reltive move: I get the stop cmd ,stoping...";
        break;
        
        case 1: //指令状态

        this->visual_pose_target_ = req.pose; //相对机器人底盘的期望运动偏置
        for (int i = 0; i < 3; i++)
        {
            err_visual_min_[i] = req.err[i];
        }
        visual_state_ = RUN;
            respon.success = true;
            respon.msg="reltive move: I get the target cmd ,runing...";
        break;

        case 2: //查询状态
        switch (visual_state_)
        {
        case WAIT:
            respon.success = true;
            respon.msg = "reltive move:Waiting target";
            break;
        case RUN:
            respon.success = false;
            respon.msg = "reltive move: Runing, wait some time";
            break;
        case INTE_STOP:
            respon.success = false;
            respon.msg = "reltive move: interupting ...";
            break;
        case COMPLATE:
            respon.success = true;
            respon.msg = "reltive move: acomplish move wait next target";
            break;
        case OUT:
            respon.success = true;
            respon.msg = "reltive move: lost ar target maybe ar is lost, you can resend the target ";
            break;
        default:
            break;
        }

        
        }
        
        return true;
    }


    /**
     * @brief 视觉跟踪主线程
     * 
     * @return true 
     * @return false 
     */
    bool reltive_move::move_visual()
    {

        xj_dy_ns::PID_controller pid_tmp[3];
        // kp ki kd maxoutput
        pid_tmp[0].init(0.08,0.0001,0,0.1);
        pid_tmp[1].init(0.08,0.0001,0,0.1);
        pid_tmp[2].init(0.1,0.0001,0,0.1);

        this->visual_state_ = WAIT;

        int pub_count_zero;
        //记录中断之后发布0速度的次数，如果发了两次，就不发了


        while (ros::ok())
        {
        geometry_msgs::Twist cmd_vel;   //机器人的指令速度

        switch (visual_state_)
        {
        case WAIT:
            break;
        case INTE_STOP:
        pub_count_zero =0;
            // 接收到中断指令，直接进入程序结尾停止运行
            break;
        case OUT:
        pub_count_zero =0;
            break;
        case RUN:
        pub_count_zero =0;
        // 当srv改变了状态为RUN，那么就进入跟踪程序

        // this->left_ar_.id = req.left_ar_id;
        // this->right_ar_.id = req.right_ar_id;
        // 如果都接受到了，那么就开始跟踪，否则就停止运动
        // 等待接受到信号

        int count_resive_visual = 0;
        while (visual_have_target_ == false )
        {
            sleep(1.0);
            count_resive_visual++;
            if (count_resive_visual>=20)
            {
                visual_state_ = OUT;//状态切换为出边界
                break;
            }
            if (ros::ok()==false)
            {
                //按ctrl+c中止运行
                visual_state_ = INTE_STOP;
                break;
            }
            // 等待接收视觉信息
        std::cout<<"count_resive_visual:"<<count_resive_visual<<std::endl;   
        }

        if (visual_state_ == OUT || visual_state_ == INTE_STOP)
        {
           break;//如果出边界了，或者节点挂了，跳出本次循环
        }
        
        // 当接受到信号之后就可以继续向下走

        double err[3];//定义误差
        
        // 首次计算ar码位姿
        
        // 首次计算误差
        err[0] = visual_pose_target_.x - visual_pose_now_.x;
        err[1] =  visual_pose_target_.y - visual_pose_now_.y ;

        // 姿态误差换成了左边减去右边，那么就应该和前面的反过来
        err[2] =  visual_pose_target_.theta - visual_pose_now_.theta ;

        //  运动的代码
        ros::Duration time;
        time.fromSec(0.1);

        sleep(0.5);

        int tmp_cont =0;
        int leave_boundary_cont =0;

        // 进入跟踪运动
        while ((ros::ok())&& tmp_cont<10)
        {
            if (feedback_state_ == INTE_STOP)//如果接受的停止指令，直接跳出控制环
            {
                break;
            }
            

            //检查是否还有这个坐标系
            if (visual_have_target_ == false)
            {
                leave_boundary_cont++;
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z= 0.0;
                // 如果不存在ar码的坐标系了，那就跳出循环，并返回跳出边界
                if (leave_boundary_cont>30)
                {
                    visual_state_ = OUT;//当前状态为中途跳出边界
                break;
                }
                else{
                    continue;
                }
            }else
            {
                leave_boundary_cont = 0;//如果又检测到了，数数停止，重新计数
            }
            
            //判断误差是否符合要求
            if ((fabs(err[0])<err_visual_min_[0]) && (fabs(err[1])<err_visual_min_[1]) && (fabs(err[2])<err_visual_min_[2]))
            {
                tmp_cont++;
            }else
            {
                tmp_cont = 0;
            }
            

            cmd_vel.linear.x = pid_tmp[0].PID(-err[0]);
            //测试
            cmd_vel.linear.y = pid_tmp[1].PID(-err[1]);
            cmd_vel.angular.z = pid_tmp[2].PID(-err[2]);
            std::cout<<"cmd_vel="<<cmd_vel<<std::endl;
            pub_cmd_vel_.publish(cmd_vel);
            
            time.sleep();

            err[0] = visual_pose_target_.x - visual_pose_now_.x;
            err[1] = visual_pose_target_.y - visual_pose_now_.y;

            // 姿态误差换成了左边减去右边，那么就应该和前面的反过来
            err[2] =  visual_pose_target_.theta - visual_pose_now_.theta;
            std::cout<<"visual_pose_target_.x"<< visual_pose_target_.x<<",,,reltive_pose.x = "<<visual_pose_now_.x<<std::endl;
            std::cout<<"visual_pose_target_.y"<<visual_pose_target_.y<<",,,reltive_pose.y = "<<visual_pose_now_.y<<std::endl;
            std::cout<<"visual_pose_target_.theta = "<<visual_pose_target_.theta<<",,,reltive_pose.yaw = "<<visual_pose_now_.theta<<std::endl;
        }
        visual_state_ = WAIT;
        break;//这个break是switch的break
        }//switch的括号

        

        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;


        if (pub_count_zero<2)
        {
            pub_cmd_vel_.publish(cmd_vel);
            pub_count_zero++;
        }
        //延时
        usleep(500*1000);
        }
        return true;
    }


    /**
     * @brief 主线程循环检测状态等
    */
    bool reltive_move::move()
    {
        // 跟踪pid初始化
        xj_dy_ns::PID_controller pid_tmp[3];

        pid_tmp[0].init(0.8,0.0001,0);
        pid_tmp[1].init(0.8,0.0001,0);
        pid_tmp[2].init(0.1,0.0001,0);

        // std::cout<<"进入了回调函数"<<std::endl;

        
    feedback_state_ = WAIT;
    int pub_count_zero = 0;
    while (ros::ok())
    {
    geometry_msgs::Twist cmd_vel;   //机器人的指令速度

        // std::cout<<"reltiv move当前状态为:"<<feedback_state_<<std::endl;
        
    // 等待命令
        switch (feedback_state_)
        {
        case WAIT:
        std::cout<<"切换至等待状态"<<std::endl;
            break;
        case INTE_STOP:
        std::cout<<"切换至中途停之状态"<<std::endl;

        pub_count_zero =0;
            // 接收到中断指令，直接进入程序结尾停止运行
            break;
        case OUT:
        std::cout<<"切换至OUT状态"<<std::endl;

        pub_count_zero =0;
            break;
        case RUN:
        std::cout<<"运行状态"<<std::endl;
        pub_count_zero =0;

        // this->left_ar_.id = req.left_ar_id;
        // this->right_ar_.id = req.right_ar_id;
        left_ar_.if_resive = false;
        right_ar_.if_resive = false;
        move_flag_ = true;//可以开始运动了,ar码开始接收信息

        // 如果都接受到了，那么就开始跟踪，否则就停止运动
        // 等待接受到信号

        int count_resive_ar = 0;
        while ((left_ar_.if_resive != true || right_ar_.if_resive != true) )
        {
            sleep(1.0);
            count_resive_ar++;
            if (count_resive_ar>=20)
            {
                feedback_state_ = OUT;//状态切换为出边界
                break;
            }
            if (ros::ok()==false)
            {
                //按ctrl+c中止运行
                feedback_state_ = INTE_STOP;
                break;
            }
        std::cout<<"无ar码count_resive_ar:"<<count_resive_ar<<std::endl;

            
            
        }
        if (feedback_state_ == OUT || feedback_state_ == INTE_STOP)
        {
           break;//如果出边界了，或者节点挂了，跳出本次循环
        }
        
        // 当接受到信号之后就可以继续向下走

        double err[3];//定义误差
        
        // 首次计算ar码位姿
        geometry_msgs::Pose2D pose_now =  trans_2ar(left_ar_,right_ar_);
        
        // 首次计算误差
        err[0] = target_pose_.x - pose_now.x;
        err[1] = target_pose_.y - pose_now.y;
        err[2] = target_pose_.theta - pose_now.theta;

        //  运动的代码
        ros::Duration time;
        time.fromSec(0.1);

        sleep(0.5);

        int tmp_cont =0;
        int leave_boundary_cont =0;

        while ((ros::ok())&& tmp_cont<10)
        {
            if (feedback_state_ == INTE_STOP)//如果接受的停止指令，直接跳出控制环
            {
                break;
            }
            

            //检查是否还有这个坐标系
            if (left_ar_.if_resive == false || right_ar_.if_resive == false)
            {
                leave_boundary_cont++;
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z= 0.0;
                // 如果不存在ar码的坐标系了，那就跳出循环，并返回跳出边界
                if (leave_boundary_cont>20)
                {
                    feedback_state_ = OUT;//当前状态为中途跳出边界
                break;
                }
                else{
                    continue;
                }
                std::cout<<"leave_boundary_cont = "<<leave_boundary_cont<<std::endl;
            }else
            {
                leave_boundary_cont = 0;//如果又检测到了，数数停止，重新计数
            }
            
            //判断误差是否符合要求
            if ((abs(err[0])<err_min_[0]) && (abs(err[1])<err_min_[1]) && (abs(err[2])<err_min_[2]))
            {
                tmp_cont++;
            }else
            {
                tmp_cont = 0;
            }
            
            if (leave_boundary_cont ==0)
            {
            cmd_vel.linear.x = pid_tmp[0].PID(-err[0]);
            //测试
            cmd_vel.linear.y = pid_tmp[1].PID(-err[1]);
            cmd_vel.angular.z = pid_tmp[2].PID(-err[2]);
            }
            else{
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z= 0.0;
            }
            

            std::cout<<"cmd_vel="<<cmd_vel<<std::endl;
            pub_cmd_vel_.publish(cmd_vel);
            
            time.sleep();

            err[0] = target_pose_.x - pose_now.x;
            err[1] = target_pose_.y - pose_now.y;
            err[2] = target_pose_.theta - pose_now.theta;


            pose_now =  trans_2ar(left_ar_,right_ar_);

            for (int i = 0; i < 3; i++)
            {
                std::cout<<"err"<<i<<"="<<err[i]<<std::endl;
            }
            // std::cout<<"reltive_pose.yaw = "<<reltive_pose.yaw<<std::endl;
        }
        feedback_state_ = WAIT;
        break;//这个break是switch的break

        }//switch的括号

        

        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;


        if (pub_count_zero<2)
        {
            pub_cmd_vel_.publish(cmd_vel);
            pub_count_zero++;
        }
        


        

        //延时
        usleep(500*1000);
        move_flag_=false;//将标志位设置为 否，接收到数据不再读取
    }    

    return true;
    }

    /**
     * @brief 通过两个ar码中心来返回两个ar码连线中心的姿态和相对位置
     * 
     * @param left_ar 左侧ar码
     * @param right_ar 右侧ar码
     * @return geometry_msgs::Pose2D 返回两者中心点以及姿态 
     */
    geometry_msgs::Pose2D reltive_move::trans_2ar(const Target_ar &left_ar,const Target_ar &right_ar)
    {
        geometry_msgs::Pose2D reltive_pose;
        reltive_pose.x = (left_ar.x +right_ar.x)/2.0;
        reltive_pose.y = (left_ar.y +right_ar.y)/2.0;
        double xerr = right_ar.x - left_ar.x;
        double yerr = right_ar.y - left_ar.y;

        // 计算角度
        reltive_pose.theta = atan(-xerr/yerr);
        return reltive_pose;
    }

    bool reltive_move::get_targetpose(agv_msg::reltive_pose_ar_get::Request &req, 
                    agv_msg::reltive_pose_ar_get::Response &respon)
    {
        this->left_ar_.id = req.left_ar_id;
        this->right_ar_.id = req.right_ar_id;
        ;
        ros::Duration time;
        time.fromSec(0.01);
        int tmp = 100;
        move_flag_ = true;
        sleep(1.0);
        geometry_msgs::Pose2D pose;
        for (int i = 0; i < 100; i++)
        {
            pose = this->trans_2ar(left_ar_,right_ar_);
            respon.pose.theta += pose.theta/100.0;
            respon.pose.x += pose.x/100.0;
            respon.pose.y += pose.y/100.0; 
            time.sleep();

        }
        respon.success = true;
        respon.msg = "return target success";
        return true;
    }

    /**
     * @brief 视觉点动回调函数
     * 
     * @param req 
     * @param respon 
     * @return true 
     * @return false 
     */
    bool reltive_move::TrackCallback_pointmove_visual(agv_msg::visual_point_move::Request &req,
                                            agv_msg::visual_point_move::Response &respon)
    {


                // 0是停止
        switch (req.state)
        {
            case 0: //停止
                point_move_struct_.state = INTE_STOP;//中断停止
                respon.success = true;
                respon.msg="reltive move: I get the stop cmd ,stoping...";
            break;
            
            case 1: //指令状态
            {
                // 将状态切换成run
                point_move_struct_.visual_d = req.target_pose;
                point_move_struct_.visual_now = req.feed_back_pose;

                tf::StampedTransform tf_w2camera;

                try
                {
                    //监听turtle2和turtle1的坐标变换
                    listener_.waitForTransform("/odom_com", "/camera1_link", ros::Time(0), ros::Duration(3.0));
                    listener_.lookupTransform("/odom_com", "/camera1_link", ros::Time(0), tf_w2camera);

                }
                catch(tf::TransformException &ex)
                {
                    ROS_ERROR("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    break;
                }
                // 获取到tf变换之后，要睡一会
                sleep(0.1);


                //摄像头到卷料的tf变换,摄像头传感出来的
                tf::Transform tf_cam2juan;
                // 设置旋转
                tf_cam2juan.setRotation(tf::createQuaternionFromYaw(point_move_struct_.visual_now.theta));
                // 设置移动向量
                tf::Vector3 tf_cam2juan_translation(point_move_struct_.visual_now.x,
                                                    point_move_struct_.visual_now.y,
                                                    0.0);
                // 将移动加进来
                tf_cam2juan.setOrigin(tf_cam2juan_translation);


                // 期望机器人行走目标到卷料的坐标变换
                tf::Transform tf_target2juan;
                // 设置旋转
                tf_target2juan.setRotation(tf::createQuaternionFromYaw(point_move_struct_.visual_d.theta));
                // 设置移动
                tf::Vector3 tf_target2juan_translation(point_move_struct_.visual_d.x,
                point_move_struct_.visual_d.y,
                0.0);
                tf_target2juan.setOrigin(tf_target2juan_translation);


                tf::Transform tf_w2target;
                tf_w2target = (tf_w2camera*tf_cam2juan)*(tf_target2juan.inverse());

                // 将世界坐标系下的目标位置储存下来
                point_move_struct_.w2target = tf_w2target;


                tf::StampedTransform tf_w2target_stamped2;
                tf_w2target_stamped2.child_frame_id_ = "target_pose";
                tf_w2target_stamped2.frame_id_ = "odom_com";
                tf_w2target_stamped2.setOrigin(tf_w2target.getOrigin()); 
                tf_w2target_stamped2.setRotation(tf_w2target.getRotation());
                
                tf_broadcaster_.sendTransform(tf_w2target_stamped2);

                point_move_struct_.state = RUN;
                respon.success = true;
                respon.msg="reltive move: I get the target cmd ,runing...";

            break;
            }

            case 2: //查询状态
            switch (point_move_struct_.state)
            {
            case WAIT:
                respon.success = true;
                respon.msg = "reltive move:Waiting target";
                break;
            case RUN:
                respon.success = false;
                respon.msg = "reltive move: Runing, wait some time";
                break;
            case INTE_STOP:
                respon.success = false;
                respon.msg = "reltive move: interupting ...";
                break;
            case COMPLATE:
                respon.success = false;
                respon.msg = "reltive move: acomplish move wait next target";
                break;
            case OUT:
                respon.success = false;
                respon.msg = "reltive move: lost ar target maybe ar is lost, you can resend the target ";
                break;
            default:
                break;
            }
        }
        return true;
    }//函数结束


    /**
     * @brief 视觉点动模式 一段时间进来一次
     * 
     */
    void reltive_move::move_point_visual()
    {

        geometry_msgs::Twist cmd_vel;   //机器人的指令速度
        cmd_vel.angular.z = 0.0;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        // pub_cmd_vel_.publish(cmd_vel);
        switch (this->point_move_struct_.state)
        {
            case NO_INIT:
                break;
            case WAIT:

                break;
            case RUN:
                {

                tf::StampedTransform tf_w2camera;
                try
                {
                    //监听turtle2和turtle1的坐标变换
                    listener_.waitForTransform("/odom_com", "/camera1_link", ros::Time(0), ros::Duration(3.0));
                    listener_.lookupTransform("/odom_com", "/camera1_link", ros::Time(0), tf_w2camera);

                }
                catch(tf::TransformException &ex)
                {   point_move_struct_.state = INTE_STOP;
                    ROS_ERROR("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    break;
                }
                    
                    // 计算cam2target，这个算出来就是误差了。pid是正向的
                    tf::Transform cam2target = tf_w2camera.inverse() * point_move_struct_.w2target; 
                    tf::Matrix3x3 cam2target_rotation(cam2target.getRotation());
                    double rpy[3];
                    cam2target_rotation.getRPY(rpy[0],rpy[1],rpy[2]);
                    
                    cmd_vel.linear.x = point_move_struct_.pid[0].PID(cam2target.getOrigin().x());
                    cmd_vel.linear.y = point_move_struct_.pid[1].PID(cam2target.getOrigin().y());
                    cmd_vel.angular.z = point_move_struct_.pid[2].PID(rpy[2]);

                    if ( (fabs(cam2target.getOrigin().x())>point_move_struct_.max_err[0]) ||
                    (fabs(cam2target.getOrigin().y())>point_move_struct_.max_err[1]) ||
                    (fabs(rpy[2])>point_move_struct_.max_err[2]) )
                    {
                        this->pub_cmd_vel_.publish(cmd_vel);
                    }
                    else{
                    cmd_vel.angular.z = 0.0;
                    cmd_vel.linear.y = 0.0;
                    cmd_vel.linear.x = 0.0;
                        pub_cmd_vel_.publish(cmd_vel);
                        point_move_struct_.state = COMPLATE;
                    }
                    
                break;
                }
            case INTE_STOP:
                std::cout << "视觉相对点动运动中断"<<std::endl;
                cmd_vel.angular.z = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.x = 0.0;
                pub_cmd_vel_.publish(cmd_vel);

                sleep(1.0);
                // 转换成wait
                point_move_struct_.state = WAIT;
                break;
            case COMPLATE:
                cmd_vel.angular.z = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.x = 0.0;
                pub_cmd_vel_.publish(cmd_vel);
                sleep(1.0);
                point_move_struct_.state = WAIT;
                break;
            case OUT:
                cmd_vel.angular.z = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.x = 0.0;
                pub_cmd_vel_.publish(cmd_vel);
                sleep(1.0);
                point_move_struct_.state = WAIT;
                break;
            default:
                break;
        }
        
    }

    /**
     * @brief 拥有各种模式的主线程，更新了版本
     * 
     */
    void reltive_move::run()
    {
        ros::Rate period(20);
        while (ros::ok())
        {
            switch (this->Visual_mode)
            {
            case POINT_MOVE : 
                //点动模式下
                move_point_visual();
                break;
            default:
                break;
            }
            period.sleep();
        }
        ;
    }
}
