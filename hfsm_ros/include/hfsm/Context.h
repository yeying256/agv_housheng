#ifndef _CONTEXT_H_
#define _CONTEXT_H_
#include "Event.h"
#include <string>
#include <unordered_map>
#include <thread>
#include <ros/ros.h>
#include <agv_msg/Button.h>
#include <agv_msg/grab_agv.h>

class State;
enum EventS
{
	next_ = 0,
	goback_,
	stop_,
	unlock_,
	telecontrol_,
	auto_,
	shutdown_,
	checkheart_,
	go_
};

struct goal_data
{
	float set_x;
	float set_y;
	float orientation;
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
	};

	~Context();

	bool Start(std::string name);

	State *CreateState(State *state, std::string name, std::string father_name = "");

	void Update();

	void SendEvent(EventData event_data);

	// 获取当前状态名称
	std::string GetCurStateName();

	void ButtonCallback(const agv_msg::Button::ConstPtr &msg);

	ros::NodeHandle n;
	ros::ServiceClient client;
	agv_msg::grab_agv srv;

private:
	// 状态转换
	void TransForState(std::string name);

	// 发送事件
	void RecursiveSend(NodeState &node_state, EventData &event_data);

	std::unordered_map<std::string, NodeState> _states;
	NodeState _cur_node_state;
	std::string _cur_name;

	std::string _root_name;
};

#endif
