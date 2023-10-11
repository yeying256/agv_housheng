#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "Eigen/Eigen"
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>

#include "agv_msg/reltive_pose.h"


namespace xj_control_ns
{
    class reltive_move
    {
    private:
        private:
        ros::NodeHandle n;
        ros::Subscriber arpos_sub_;
        bool move_flag_;
        double m_p;
    //    double m_i;
    //    double m_d;
        double m_err;
        int target_ar_id_;
        double confidence_;//ar码的可信度

        ros::ServiceServer ar_target_Server_;



        geometry_msgs::Pose ar_reltive_;//变换后的ar姿态
    public:
        reltive_move(/* args */);

        //对ar码做一个坐标变换
        geometry_msgs::Pose trans_arpose(const geometry_msgs::Pose&  ar_reltive);
        void pose_callback(const ar_track_alvar_msgs::AlvarMarkers& pose_msgs);
        ~reltive_move();

        bool TrackCallback(agv_msg::reltive_pose::Request &req, 
                            agv_msg::reltive_pose::Response &respon);


    };
    

    
}