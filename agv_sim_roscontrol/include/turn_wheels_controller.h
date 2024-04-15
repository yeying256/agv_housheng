#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>





#include <hardware_interface/hardware_interface.h>
#include <controller_interface/multi_interface_controller.h>

#include "robot_dynamic.h"

#include "agv_turn_wheel.h"
// #include <matlogger2/matlogger2.h>
// #include <matlogger2/utils/mat_appender.h>



#include "mean_filter.h"

#include "agv_msg/hw_srv.h"
#include "agv_msg/Test_msg.h"







namespace xj_control_ns
{
    class turn_wheels_controller:
    public controller_interface::MultiInterfaceController
    <hardware_interface::VelocityJointInterface> 
    {
    private:
        /* data */
        int wheel_num;

        std::vector<hardware_interface::JointHandle> turn_wheel_joint_handles_;
        xj_dy_ns::agv_turn_wheel agv_cal_;
        ros::Subscriber sub_cmd_vel;
        ros::ServiceServer service_goback_zero_;
        Eigen::Vector3d xyw_cmd_;
        Eigen::Vector2d turn_theta_,omega_turn_now_,omega_wheel_now_;
        Eigen::Vector2d pos_turn_last_,pos_wheel_last_,pos_wheel_now_;

        // XBot::MatLogger2::Ptr logger;
        ros::Time time_starting_;
        std::vector<Eigen::VectorXd> log_xyw_cmd_,log_cmd_turn_vel_,log_cmd_wheel_vel_;
        bool log_flag_;

        //播报tf
        tf::TransformBroadcaster odom_broadcaster_;
        //播报里程计信息
        // ros::Publisher odom_pub_;
        realtime_tools::RealtimePublisher<nav_msgs::Odometry> odom_pub_;
        //tf信息
        geometry_msgs::TransformStamped odom_tf_;
        //odom信息
        nav_msgs::Odometry odom_;

        xj_dy_ns::MeanFilter cmd_filter_;
        bool flag_zero_;
        bool flag_sim_;


        ros::Publisher test_msg_pub_;

        // 最大角速度
        double max_vel_ = 15.0;
        
        
        

    public:
        turn_wheels_controller(/* args */);
        ~turn_wheels_controller();
        bool init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& nh);
        void starting(const ros::Time& /*time*/);
        void update(const ros::Time& time, const ros::Duration& period);

        void cb_target_pose(const geometry_msgs::Twist& msg);

        void log_thread(int size);
        // bool doReq(demo03_server_client::AddInts::Request& req,
        //   demo03_server_client::AddInts::Response& resp)
        bool zero_srv_cb(agv_msg::hw_srv::Request& req,
                        agv_msg::hw_srv::Response & resp);
    };
    

    

}