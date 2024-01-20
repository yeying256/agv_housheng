#ifndef _CONTEXT_H_
#define _CONTEXT_H_
#include "Event.h"
#include <string>
#include <unordered_map>
#include <thread>
#include <ros/ros.h>
#include <agv_msg/Button.h>
#include <agv_msg/grab_agv.h>
#include <agv_msg/YoloResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <hfsm/SendGoal.h>

#define OBJ_LEN 5.0

class State;
enum EventS
{
	next_ = 0,
	goback_,
	stop_,
	unlock_,
	lift_,
	grab_reset_,
	task_,
	grab_open_,
	telecontrol_,
	auto_,
	shutdown_,
	checkheart_,
	go_,
	put_,
	automove_grabmove_,
	photo_
};

struct goal_data
{
	float set_x;
	float set_y;
	geometry_msgs::Quaternion orientation;
};

struct grab_move
{
	double width;	//单位mm
	double heigh;	//单位mm
	bool if_width;	//true
};

struct ar_data
{
	int pre_height;
	int pre_width;
	int height;
	int width;
	int workbench_id;
};

/**
 * @brief yolo检测出来的目标状态 有卷料的2d位姿还有卷料的长度
 * 
 */
struct Object
{
	geometry_msgs::Pose2D pose;
	float length = 0;
};


struct NodeState
{
	NodeState &operator=(const NodeState &n)
	{
		_state = n._state;
		_father_name = n._father_name;
		return *this;
	}
	State *_state;
	std::string _father_name;
};

class Context
{
public:
	friend class State;

	Context(ros::NodeHandle nh) : n(nh)
	{
		client = n.serviceClient<agv_msg::grab_agv>("robot_control");
		// visual_feedback_pub = n.advertise<agv_msg::Visual_fb>("/visual_pose_fb", 1);
	};

	~Context();

	bool Start(std::string name);

	State *CreateState(State *state, std::string name, std::string father_name = "");

	void Update();

	void SendEvent(EventData event_data);

	// 获取当前状态名称
	std::string GetCurStateName();

	//按键回调
	void buttonCallback(const agv_msg::Button::ConstPtr &msg);

	//状态回调。action的回调。
	void statusCallback(const move_base_msgs::MoveBaseActionResult &msg);

	void yoloCallback(const agv_msg::YoloResult &msg);

	// 状态转换
	void TransForState(std::string name);

	ros::NodeHandle n;

	ros::ServiceClient client;//夹爪运动的client0:急停；1：AGV回零；2：夹爪回零；3：两部分一起运动；4：AGV单独运动；5：夹爪单独运动

	agv_msg::grab_agv srv;//夹爪运动的数据

	NavState nav;//调用此对象里面有成员函数setgaol可以发布指令

	bool single_flag = 0;

	bool detector_flag = 0;

	int take_flag = 0;

	int put_flag = 0;

	int nav_flag = 0;//flag = 0的时候是导航结束，到位置了，可以发布下一个命令了

	bool run_ = true; 

	std::vector<Object> object_list;//yolo检测出来的目标列表

private:
	// 发送事件
	void RecursiveSend(NodeState &node_state, EventData &event_data);

	std::unordered_map<std::string, NodeState> _states;

	NodeState _cur_node_state;

	std::string _cur_name;

	std::string _root_name;

	ros::Publisher visual_feedback_pub;
};

#endif
