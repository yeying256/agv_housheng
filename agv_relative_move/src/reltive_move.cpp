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
        0,1,0,
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
        if (this->move_flag_ == true)//如果是让运动，才开始下列的操作
        {
            for (int i = 0; i < marks_msgs.markers.size(); i++)//遍历所有识别到的ar码
            {
                if (marks_msgs.markers[i].id==this->target_ar_id_)//如果是需要跟踪的id,那么把信息加载下来
                {
                    this->confidence_ =  marks_msgs.markers[i].confidence;
                    ar_reltive_ = this->trans_arpose(marks_msgs.markers[i].pose.pose);//将处理好的pose加载到内部参数

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
        for (int i = 0; i < 3; i++) //初始化移动机器人的位置跟踪pid
        {
            pid_tmp[i].init(10,0,0);
        }


        geometry_msgs::Twist cmd_vel;   //机器人的指令速度
        double hight = req.high;//机械手臂的宽和高
        double width = req.width;
        this->target_ar_id_ = req.target_ar_id;
        move_flag_ = true;//可以开始运动了

        double err[3];
        err[0] = target_pose.x - this->ar_reltive_.position.x;
        err[1] = target_pose.y - this->ar_reltive_.position.y;
        err[2] = target_pose.theta - reltive_pose.yaw;

        double err_all = 0;
        for (int i = 0; i < 2; i++)
        {
            err_all+=abs(err[i]);
        }//计算总体误差
        
        
        //  运动的代码

        while (err_all>0.001 || abs(err[2])>0.005)
        {
            cmd_vel.linear.x = pid_tmp[0].PID(err[0]);
            cmd_vel.linear.y = pid_tmp[1].PID(err[1]);
            cmd_vel.angular.z = pid_tmp[2].PID(err[2]);
            pub_cmd_vel_.publish(cmd_vel);
        }
        
        //  运动的代码

        move_flag_=false;
        respon.success = true;
        respon.msg = "相对运动结束";
        return true;
    }

    

}