#include <hfsm/Context.h>
#include <hfsm/State.h>
#include <iostream>
#include <actionlib_msgs/GoalStatus.h>
#include <numeric>
#include <math.h>

Context::~Context()
{
	if (_cur_node_state._state != nullptr)
	{
		_cur_node_state._state->stop();
	}

	for (auto iter : _states)
	{
		if (iter.second._state)
		{
			delete iter.second._state;
			iter.second._state = nullptr;
		}
	}
	_states.clear();
	system("rosnode kill /map_server");
}

bool Context::Start(std::string name)
{
	std::unordered_map<std::string, NodeState>::iterator iter_map = _states.find(name);
	if (iter_map != _states.end())
	{
		_cur_node_state = iter_map->second;
		_cur_name = iter_map->first;
		iter_map->second._state->start();
	}
	return false;
}

State *Context::CreateState(State *state, std::string name, std::string father_name)
{
	NodeState node_state;
	node_state._state = state;
	node_state._state->SetContext(this);
	node_state._father_name = father_name;
	_states[name] = node_state;
	return state;
}

void Context::Update()
{
	_cur_node_state._state->update();
}

void Context::SendEvent(EventData event_data)
{
	RecursiveSend(_cur_node_state, event_data);
}

std::string Context::GetCurStateName()
{
	return _cur_name;
}

void Context::RecursiveSend(NodeState &node_state, EventData &event_data)
{
	EventDeal event_deal = node_state._state->RunEventFunc(event_data);
	if (event_deal == keep_on && !node_state._father_name.empty())
	{
		std::unordered_map<std::string, NodeState>::iterator iter_map = _states.find(node_state._father_name);
		if (iter_map != _states.end())
		{
			RecursiveSend(iter_map->second, event_data);
		}
	}
}

void Context::TransForState(std::string name)
{
	std::string str_name = std::string(name);
	std::unordered_map<std::string, NodeState>::iterator iter_map = _states.find(str_name);
	if (iter_map != _states.end())
	{
		_cur_node_state._state->stop();
		_cur_node_state = iter_map->second;
		_cur_name = iter_map->first;
		_cur_node_state._state->start();
	}
}

void Context::buttonCallback(const agv_msg::Button::ConstPtr &msg)
{
	for (int n = 0; n < msg->button_type.size(); n++)
	{
		ROS_INFO("Current State is %s", this->GetCurStateName().c_str());
		ROS_INFO("response %s request", msg->button_func[n].c_str());
		switch (msg->button_type[n])
		{
		case next_: // 按A 下一步
		{
			this->Update();
			break;
		}
		// case goback_:
		// {
		// 	this->TransForState("Auto");
		// 	geometry_msgs::Quaternion q;
		// 	q.w = 1;
		// 	q.z = 0;
		// 	q.y = 0;
		// 	q.x = 0;
		// 	EventData e = EventData((int)go_);
		// 	goal_data start = {0.0, 0.0, q};
		// 	e.SetData(&start);
		// 	this->SendEvent(e);
		// 	break;
		// }
		case stop_:
		{
			// EventData e = EventData((int)unlock_);
			// this->SendEvent(e);
			client.waitForExistence();
			srv.request.status = 0;

			EventData e = EventData((int)stop_);
			this->SendEvent(e);

			if (client.call(srv))
			{
				ROS_INFO("EMERGENCY STOP!");
				this->TransForState("Lock");
			}
			else
				ROS_ERROR("STOP FAILED!");

			break;
		}
		case unlock_:
		{
			// EventData e = EventData((int)unlock_);
			// this->SendEvent(e);
			std::string current_state = this->GetCurStateName();
			if (current_state == "Idle")
			{
				client.waitForExistence();
				srv.request.status = 4;
				if (client.call(srv))
				{
					ROS_INFO("AGV UNLOCK!");
					this->TransForState("Telecontrol");
				}
				else
					ROS_ERROR("UNLOCK FAILED");
				break;
			}
			else
				ROS_ERROR("StateTrans error, current state is %s", current_state);
		}
		case grab_reset_:
		{
			client.waitForExistence();
			srv.request.status = 2;
			if (client.call(srv))
			{
				ROS_INFO("grab_result = : %d", srv.response.grab_result);
				if (srv.response.grab_result == 1)
				{
					ROS_INFO("GRAB READY!");
					this->TransForState("Telecontrol");
				}
				else
					ROS_INFO("GRAB RESET FAILED!");
			}
			break;
		}
		case lift_:
		{
			client.waitForExistence();
			srv.request.status = 5;
			srv.request.high = 420;
			srv.request.width = 0;
			if (client.call(srv))
			{
				ROS_INFO("GRAB MOVE!");
				// if (srv.response.grab_result == 1)
				// {
				// 	ROS_INFO("GRAB LIFT OK!");
				this->TransForState("Telecontrol");
				// }
			}
			else
				ROS_INFO("GRAB LIFT FAILED!");
			break;
		}
		case task_: // A健 按着右边的按键
		{
			// if (detector_flag)

			this->TransForState("Auto");
			geometry_msgs::Quaternion q;
			q.w = 0.01746241603211958;
			q.z = -0.9998475203881446;
			q.y = 0;
			q.x = 0;
			EventData e = EventData((int)go_);
			goal_data start = {-4.487979013164646, 3.4279136259250427, q};
			e.SetData(&start);
			this->SendEvent(e);
			break;
		}
		case grab_open_: // B
		{
			client.waitForExistence();
			srv.request.status = 5;
			srv.request.high = 212;
			srv.request.width = 305;
			if (client.call(srv))
			{
				ROS_INFO("GRAB MOVE!");
				// if (srv.response.grab_result == 1)
				// {
				// 	ROS_INFO("GRAB LIFT OK!");
				this->TransForState("Telecontrol");
				// }
			}
			else
				ROS_INFO("GRAB OPEN FAILED!");
			// grab_move(target_length,pre_height,true);
			break;
		}
		}
	}
}

/**
 * @brief
 *
 * @param msg
 */
void Context::statusCallback(const move_base_msgs::MoveBaseActionResult &msg)
{
	if (msg.status.status == 3)
	{
		std::cout << "the goal was achieved successfully!" << std::endl;

		this->nav_flag = 0; // 导航结束，发布下一个命令

		// this->TransForState("Adjust");//导航结束就跳转到调整命令？？？？
	}
}

/**
 * @brief Yolo信息的回调函数
 *
 * @param msg 储存着结果，有目标数量
 */
void Context::yoloCallback(const agv_msg::YoloResult &msg)
{
	// std::cout << "detector pub!" << std::endl;
	// ros::Time t = ros::Time::now();
	// ros::Time t_error = t - ros::Duration(0.05);
	// detector_flag = (msg.header.stamp > t_error) ? true : false;
	if (msg.object_num == 0)
	{
		detector_flag = false;
		return;
	}
	int n = msg.position.size();

	object_list.clear();
	switch (detector_type)
	{
	case sleep_:
		break;

	case detecting_:
	{
		float sum_len = 0.0;
		for (int i = 0; i < n; i++)
		{
			Object obj;
			obj.pose = msg.position[i];
			obj.length = msg.length[i];
			sum_len += obj.length;
			object_list.push_back(obj);
		}

		detector_flag = (OBJ_LEN - sum_len < 500) ? true : false;

		break;
	}

	case feedback_:
	{
		single_flag = (n == 1) ? true : false;
		if (single_flag)
		{
			length_result_buffer.pop_front();
			length_result_buffer.push_back(msg.length[0]);
			double var = VectorVar(length_result_buffer);

			if (var < 0.5)
			{
				Object obj;
				obj.pose = msg.position[0];
				obj.length = msg.length[0];
				object_list.push_back(obj);
			}
		}
		break;
	}
	default:
		break;
	}
}

double VectorVar(const std::deque<double> &A)
{
	double sum = std::accumulate(std::begin(A), std::end(A), 0.0);
	double mean = sum / A.size();

	double variance = 0.0;
	for (uint16_t i = 0; i < A.size(); i++)
	{
		variance = variance + pow(A[i] - mean, 2);
	}
	variance = variance / A.size();
	return variance;
}
