#include "turn_wheels_controller.h"
#include <pluginlib/class_list_macros.h>

namespace xj_control_ns
{
    turn_wheels_controller::turn_wheels_controller(/* args */)
    {
        // logger = XBot::MatLogger2::MakeLogger("/home/wangxiao/learn_ws/src/agv_sim/agv_sim_roscontrol/log_data_mat/mylog.mat");//设置路径
        // auto appender = XBot::MatAppender::MakeInstance();
        // logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);//设置记录器循环记录模式

        // appender->add_logger(logger);
        // appender->start_flush_thread();
    }

    turn_wheels_controller::~turn_wheels_controller()
    {
    }

    void turn_wheels_controller::cb_target_pose(const geometry_msgs::Twist &msg)
    {

        xyw_cmd_[0] = msg.linear.x;
        xyw_cmd_[1] = msg.linear.y;
        xyw_cmd_[2] = msg.angular.z;
    }

    bool turn_wheels_controller::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh)
    {
        printf("\033[1;36;40m  init   \033[0m \n");
        std::vector<std::string> joint_names;
        hardware_interface::VelocityJointInterface *turn_wheel_jointinterface = robot_hw->get<hardware_interface::VelocityJointInterface>();
        // 获取速度接口

        int interface_num = turn_wheel_jointinterface->getNames().size();

        if (!nh.getParam("joints", joint_names))
        {
            ROS_WARN("joint num is err,or can not read the joint param,joint dof param = %ld", joint_names.size());
            return false;
        }

        if (interface_num == 4)
        {
            printf("\033[1;36;40m front %d is robot joint \033[0m \n", interface_num);
        }

        for (int i = 0; i < interface_num; i++)
        {

            this->turn_wheel_joint_handles_.push_back(turn_wheel_jointinterface->getHandle(joint_names[i]));
            // turn1 turn3 wheel1 wheel3
        }

        double lr, lf, B, R;
        nh.getParam("/agv_sim/lr", lr);
        nh.getParam("/agv_sim/lf", lf);
        nh.getParam("/agv_sim/B", B);
        nh.getParam("/agv_sim/R", R);

        
        nh.getParam("/use_sim_time",this->flag_sim_);



        // 如果是仿真环境，这个参数是true，直接回零，否则需要等待回零

        B = 0.625;
        R = 0.625;

        printf("\033[1;36;40m  /agv_sim/lr = %f lf=%f,B=%f,R=%f \033[0m \n", lr, lf, B, R);
        agv_cal_.init(lr, lf, B, R);

        // 新参数0.90547
        Eigen::Vector2d phi, b, r, a_init, l;

        // 仿真参数
        // phi<<0.90547,0.90547+M_PI;
        // b<<0.05,0.05;
        // r<<0.05,0.05;
        // l<<0.41309805131,0.41309805131;
        // a_init<<M_PI_2,M_PI_2;

        // 新机器人参数
        phi << 0.75976, (0.75976 - M_PI);
        b << 0.0625, 0.0625;
        r << 0.0625, 0.0625;
        l << 0.41379, 0.41379;
        a_init << 0, 0;

        agv_cal_.init(phi, b, r, l, a_init);

        // nh.subscribe("/cmd_vel",10,);
        // ros::Subscriber subscribe<M>(const std::string &topic, uint32_t queue_size, void (*fp)(const boost::shared_ptr<const M> &), const ros::TransportHints &transport_hints = ros::TransportHints())
        sub_cmd_vel = nh.subscribe("/cmd_vel", 10, &turn_wheels_controller::cb_target_pose, this, ros::TransportHints().reliable().tcpNoDelay());
        service_goback_zero_ = nh.advertiseService("/ifzero_srv", &turn_wheels_controller::zero_srv_cb, this);
        // 初始化里程计播报

        // odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom",50);
        odom_pub_.init(nh, "/odom", 1, false);

        cmd_filter_.init(100, 3);

        log_flag_ = true;
        if (flag_sim_ == true)
        {
            flag_zero_ = true;
        }else
        {flag_zero_ =false;}

        test_msg_pub_ = nh.advertise<agv_msg::Test_msg>("/test_data_topic",10);
        


        return true;
    }

    void turn_wheels_controller::starting(const ros::Time &time)
    {
        for (int i = 0; i < turn_theta_.size(); i++)
        {
            this->turn_theta_[i] = turn_wheel_joint_handles_[i].getPosition(); // 将角度返回
        }

        for (int i = 0; i < turn_wheel_joint_handles_.size(); i++)
        {
            turn_wheel_joint_handles_[i].setCommand(0.0);
            // 初始状态cmd_0
        }
        time_starting_ = time;
    }
    void turn_wheels_controller::update(const ros::Time &time, const ros::Duration &period)
    {
        Eigen::Vector4d vel_4_now;
        for (int i = 0; i < turn_theta_.size(); i++)
        {
            // this->turn_theta_[i]=turn_wheel_joint_handles_[i].getPosition();//将角度返回
            // this->omega_turn_now_[i]=turn_wheel_joint_handles_[i].getVelocity();//将转向电机的角速度返回
            this->omega_wheel_now_[i] = turn_wheel_joint_handles_[i + 2].getVelocity(); // 将行走电机的角速度返回
        }
        this->turn_theta_[0] = turn_wheel_joint_handles_[0].getPosition(); // 将角度返回
        this->turn_theta_[1] = turn_wheel_joint_handles_[2].getPosition(); // 将角度返回

        this->omega_turn_now_[0] = turn_wheel_joint_handles_[0].getVelocity(); // 将转向电机的角速度返回
        this->omega_turn_now_[1] = turn_wheel_joint_handles_[2].getVelocity(); // 将转向电机的角速度返回

        this->omega_wheel_now_[0] = turn_wheel_joint_handles_[1].getVelocity(); // 将行走电机的角速度返回
        this->omega_wheel_now_[1] = turn_wheel_joint_handles_[3].getVelocity(); // 将行走电机的角速度返回

        this->pos_wheel_now_[0] = turn_wheel_joint_handles_[1].getPosition(); // 将行走电机的角速度返回
        this->pos_wheel_now_[1] = turn_wheel_joint_handles_[3].getPosition(); // 将行走电机的角速度返回

        // std::cout << "\033[1;36;40m "
        //           << "turn_theta_=" << turn_theta_ << "\033[0m " << std::endl;

        // agv_cal_.update(omega_wheel_now_,omega_turn_now_,turn_theta_,period);
        if (flag_zero_ == true)
        {
            agv_cal_.update(pos_wheel_now_,
                            turn_theta_,
                            period);
        }

        // 使用微小位移更新

        // std::cout << "\033[1;36;40m "
        //           << "pos_wheel_now_=" << pos_wheel_now_ << "\033[0m " << std::endl;

        // std::cout<<"\033[1;36;40m "<<"turn_theta_="<<turn_theta_<<"\033[0m "<<std::endl;
        // std::cout<<"\033[1;36;40m "<<"vel_4_now="<<vel_4_now<<"\033[0m "<<std::endl;

        // 读取当前反馈信息,更新运动学内部参数
        agv_cal_.tf_odom_trans(this->odom_, this->odom_tf_, time, "odom", "base_footprint");

        odom_.pose.covariance = {1e-6, 0, 0, 0, 0, 0,
                                 0, 1e-6, 0, 0, 0, 0,
                                 0, 0, 1e6, 0, 0, 0,
                                 0, 0, 0, 1e6, 0, 0,
                                 0, 0, 0, 0, 1e6, 0,
                                 0, 0, 0, 0, 0, 1e-6};
        odom_.twist.covariance = {1e-6, 0, 0, 0, 0, 0,
                                  0, 1e-6, 0, 0, 0, 0,
                                  0, 0, 1e1, 0, 0, 0,
                                  0, 0, 0, 1e6, 0, 0,
                                  0, 0, 0, 0, 1e6, 0,
                                  0, 0, 0, 0, 0, 1e-6};
        // 发送消息
        odom_pub_.msg_ = odom_;
        odom_pub_.unlockAndPublish();
        //测试的tf变换
        // odom_broadcaster_.sendTransform(odom_tf_);

        Eigen::Vector2d cmd_vel_drive, cmd_vel_steer;
        // agv_cal_.Steer_Wheel_Kinematics(cmd_vel_drive,cmd_vel_steer,turn_theta_,this->xyw_cmd_);//计算逆向运动学

        Eigen::Vector3d xyw_cmd_tmp;

        xyw_cmd_tmp = cmd_filter_.Filter(xyw_cmd_);


        // agv_cal_.Inverse_Kinematics_new(cmd_vel_drive, cmd_vel_steer, turn_theta_, xyw_cmd_tmp);
        agv_cal_.Inverse_Kinematics_new3(cmd_vel_drive, cmd_vel_steer, turn_theta_, xyw_cmd_tmp);

        Eigen::Vector3d xyw_forword_cal = agv_cal_.Forward_Kinematics2(cmd_vel_drive,cmd_vel_steer,turn_theta_);

        // std::cout<<"============="<<std::endl<<"xyw_cmd_tmp = "<<xyw_cmd_tmp<<"xyw_forword = "<<xyw_forword_cal <<std::endl;


        // Eigen::Vector2d turn_theta_test;
        // Eigen::Vector3d xyw_cmd_test;
        // xyw_cmd_test<<0,0,3;
        // turn_theta_test<< 1.02,-2.11;
        // agv_cal_.Steer_Wheel_Kinematics(cmd_vel_drive,cmd_vel_steer,turn_theta_test,xyw_cmd_test);//计算逆向运动学

        Eigen::Vector4d cmd_turn_drive;
        cmd_turn_drive << cmd_vel_steer, cmd_vel_drive;
        cmd_turn_drive[0] = cmd_vel_steer[0];
        cmd_turn_drive[1] = cmd_vel_drive[0];
        cmd_turn_drive[2] = cmd_vel_steer[1];
        cmd_turn_drive[3] = cmd_vel_drive[1];

        for (int i = 0; i < turn_wheel_joint_handles_.size(); i++)
        {
            turn_wheel_joint_handles_[i].setCommand(cmd_turn_drive[i]);
            // turn_wheel_joint_handles_[i].setCommand(10);

            // std::cout<<"\033[1;36;40m "<<"cmd_turn_drive"<<i<<"="<<cmd_turn_drive[i]<<"\033[0m "<<std::endl;
        }

        // turn_wheel_joint_handles_[1].setCommand(1.0);

        // omega_turn_now_[0];

        // std::cout<<"\033[1;36;40m "<<"omega_turn_now_"<<0<<"="<<omega_turn_now_[0]<<"\033[0m "<<std::endl;

        ros::Duration time_es = time - this->time_starting_;
        if (time_es.toSec() <= 10.0)
        {
            log_xyw_cmd_.push_back(xyw_cmd_);
            log_cmd_turn_vel_.push_back(cmd_turn_drive.topRows(2));
            log_cmd_wheel_vel_.push_back(cmd_turn_drive.bottomRows(2));

            // logger->add("xyw_cmd_",xyw_cmd_);
            // logger->add("cmd_turn_vel",cmd_turn_drive.topRows(2));
            // logger->add("cmd_wheel_vel",cmd_turn_drive.bottomRows(2));
        }
        else if (log_flag_ == true)
        {
            // std::thread th_data_log(&turn_wheels_controller::log_thread,this,log_xyw_cmd_.size());
            // th_data_log.detach();
            log_flag_ = false;
        }

        agv_msg::Test_msg test_msg;
        test_msg.turn_cmd_vel =cmd_turn_drive[0];
        test_msg.turn_real_vel=omega_turn_now_[0];
        test_msg.wheel_cmd_vel = cmd_turn_drive[1];
        test_msg.wheel_real_vel = omega_wheel_now_[0];
        test_msg_pub_.publish(test_msg);

        // printf("这里是roscontrol的");
    }
    void turn_wheels_controller::log_thread(int size)
    {
        // auto logger2 = XBot::MatLogger2::MakeLogger("/home/wangxiao/learn_ws/src/agv_sim/agv_sim_roscontrol/log_data_mat/mylog");
        // logger2->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);//设置记录器循环记录模式
        // for (int i = 0; i < size; i++)
        // {
        //     logger2->add("xyw_cmd_",log_xyw_cmd_[i]);
        //     logger2->add("cmd_turn_vel",log_cmd_turn_vel_[i]);
        //     logger2->add("cmd_wheel_vel",log_cmd_wheel_vel_[i]);
        // }
    }

    /**
     * @brief 查看是否回零，回零了才能更新里程计，
     */
    bool turn_wheels_controller::zero_srv_cb(agv_msg::hw_srv::Request &req,
                                             agv_msg::hw_srv::Response &resp)
    {
        flag_zero_ = req.if_zero;
        // 将是否回零设置到私有变量中去
        resp.result = true;
        return true;
    }
}

PLUGINLIB_EXPORT_CLASS(xj_control_ns::turn_wheels_controller,
                       controller_interface::ControllerBase)
