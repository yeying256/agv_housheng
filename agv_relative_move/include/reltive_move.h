#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "Eigen/Eigen"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/thread.hpp>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "agv_msg/reltive_pose.h"
#include "agv_msg/reltive_pose_ar.h"
#include "PID_controller.h"
#include "agv_msg/reltive_pose_ar_get.h"

#include "agv_msg/reltive_pose_visual.h"
#include "agv_msg/Visual_fb.h"

#include "agv_msg/visual_point_move.h"
#include "tf_conversions/tf_eigen.h"


namespace xj_control_ns
{
    class reltive_move
    {
    private:
        private:
        ros::NodeHandle n;
        ros::Subscriber arpos_sub_;
        //视觉的subscriber
        ros::Subscriber visualPos_sub_;
        //是否接受到视觉目标的flag
        bool visual_have_target_;




        bool move_flag_,have_target_;
        double m_p;
    //    double m_i;
    //    double m_d;
        double m_err;
        int target_ar_id_;
        double confidence_;//ar码的可信度

        //使用单个ar码的位姿来搞定
        ros::ServiceServer ar_target_Server_;

        // 使用双ar码
        ros::ServiceServer ar_target_Server2_;
        ros::ServiceServer ar_target_get_;

        //使用视觉来搞定
        ros::ServiceServer visual_target_Server_;

        // 点动的视觉Service 不是视觉伺服的方法
        ros::ServiceServer visualPos_point_Server_;




        ros::Publisher pub_cmd_vel_;

        enum State{
            WAIT,//等待命令
            RUN,//正在运行 
            COMPLATE,//正常停止
            INTE_STOP,//中途停止
            OUT, //出边界了
            NO_INIT

        }feedback_state_;

        State visual_state_;

        



        geometry_msgs::Pose ar_reltive_;//变换后的ar姿态
        struct Reltive_move
        {
            double roll;
            double pitch;
            double yaw;
        }reltive_pose;

        struct Target_ar
        {
            int id;
            double x;
            double y;
            bool if_resive;
            double confidance;
        };

        struct
        {
            bool have_target = false;
            State state;
            // 期望位姿
            geometry_msgs::Pose2D visual_d;
            // 反馈的位姿
            geometry_msgs::Pose2D visual_now;
            // 在世界坐标系下的最终达到的目标位置
            tf::Transform w2target;
            // x,y,yaw的pid
            xj_dy_ns::PID_controller pid[3];
            // 最大误差
            double max_err[3];

        }point_move_struct_;
        


        //视觉信息
        geometry_msgs::Pose2D visual_pose_target_;
        geometry_msgs::Pose2D visual_pose_now_;


        

        // 两个ar码的xy数据还有id
        Target_ar left_ar_;
        Target_ar right_ar_;
        double err_min_[3];
        double err_visual_min_[3];


        xj_dy_ns::PID_controller *pid_;//没什么用

        tf::TransformListener listener_;//tf监听器
        tf::TransformBroadcaster tf_broadcaster_; //tf广播器
        geometry_msgs::Pose2D target_pose_;
        
        
        

    public:

    enum Visual_Mode{
            POINT_MOVE,
            SERVE_MODE,
            AR_MODE,
            DOUBLE_AR_MODE
        }Visual_mode;
        reltive_move(/* args */);

        //对ar码做一个坐标变换
        geometry_msgs::Pose trans_arpose(const geometry_msgs::Pose&  ar_reltive);
        void pose_callback(const ar_track_alvar_msgs::AlvarMarkers& pose_msgs);

        //视觉信息反馈
        void visual_callback(const agv_msg::Visual_fb& pose_msgs);

        ~reltive_move();

        bool TrackCallback(agv_msg::reltive_pose::Request &req, 
                            agv_msg::reltive_pose::Response &respon);
        
        //使用两个ar码来跟踪位姿的回调函数
        bool TrackCallback2(agv_msg::reltive_pose_ar::Request &req, 
                            agv_msg::reltive_pose_ar::Response &respon);
        
        //通过左右ar码来计算相对位姿
        geometry_msgs::Pose2D trans_2ar(const Target_ar &left_ar,const Target_ar &right_ar);

        bool get_targetpose(agv_msg::reltive_pose_ar_get::Request &req, 
                            agv_msg::reltive_pose_ar_get::Response &respon);
        bool TrackCallback_visual(agv_msg::reltive_pose_visual::Request &req, 
                            agv_msg::reltive_pose_visual::Response &respon);

        bool TrackCallback_pointmove_visual(agv_msg::visual_point_move::Request &req,
                                            agv_msg::visual_point_move::Response &respon);
        
        // ar码的相对运动指令
        bool move();

        // 视觉的相对运动指令
        bool move_visual();

        //视觉点动模式
        void move_point_visual();
        //初始化函数
        bool init(uint8_t mode);

        void run();



    };
    

    
}