#include "reltive_move.h"




namespace xj_control_ns
{

    /**
     * @brief 构造函数
     * 
     */
    reltive_move::reltive_move(/* args */)
    {
        //有参构造
        pid_ = new xj_dy_ns::PID_controller(20,0,0);
        //接收ar_pose
        arpos_sub_  = n.subscribe("ar_pose_marker", 10, &reltive_move::pose_callback,this);

        //接收target
        ar_target_Server_ = n.advertiseService("ar_track_target", &reltive_move::TrackCallback,this);

        //发送机器人运动速度命令
        pub_cmd_vel_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    }
    
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
            for (int i = 0; i < marks_msgs.markers.size(); i++)//遍历所有识别到的ar码
            {
                if (marks_msgs.markers[i].id==this->target_ar_id_)//如果是需要跟踪的id,那么把信息加载下来
                {
                    this->confidence_ =  marks_msgs.markers[i].confidence;
                    ar_reltive_ = this->trans_arpose(marks_msgs.markers[i].pose.pose);//将处理好的pose加载到内部参数
                    
                    std::cout<<"\033[1;32;40mar_reltive_.orientation = "<<ar_reltive_.orientation<<"\033[0m"<<std::endl;
                    std::cout<<"ar_reltive_.position = "<<ar_reltive_.position<<std::endl;


                    tf::Quaternion quat;
                    tf::quaternionMsgToTF(ar_reltive_.orientation, quat); //将相对方向转换成tf
                    tf::Matrix3x3(quat).getRPY(reltive_pose.roll, reltive_pose.pitch, reltive_pose.yaw);
                }
            }
        }
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
            cmd_vel.linear.y = pid_tmp[1].PID(-err[1]);
            cmd_vel.angular.z = pid_tmp[2].PID(-err[2]);
            std::cout<<"cmd_vel="<<cmd_vel<<std::endl;
            pub_cmd_vel_.publish(cmd_vel);
            
            time.sleep();
            err[0] = target_pose.x - this->ar_reltive_.position.x;
            err[1] = target_pose.y - this->ar_reltive_.position.y;
            err[2] = target_pose.theta - reltive_pose.yaw;

            err_all = abs(err[0])+abs(err[1]);

            for (int i = 0; i < 3; i++)
            {
                std::cout<<"err"<<i<<"="<<err[i]<<std::endl;
            }
            std::cout<<"reltive_pose.yaw = "<<reltive_pose.yaw<<std::endl;



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
        respon.success = true;//给客户端返回成功
        respon.msg = "reltive move success";    
        return true;
    }

    

}