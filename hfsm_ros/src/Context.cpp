#include <hfsm/Context.h>
#include <hfsm/State.h>
#include <hfsm/SendGoal.hpp>
#include <iostream>

Context::Context()
{
}

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
}

// ��ʼ״̬��
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

// ����һ��״̬
// [in] state ״̬������Context����ʱ���ڲ��ͷ�state
// [in] name  ״̬���ƣ�Ϊ������Ϊtypedname��ֵ
// [in] father_name ��״̬������
// [out] ����state
State *Context::CreateState(State *state, std::string name, std::string father_name)
{
	NodeState node_state;
	node_state._state = state;
	node_state._state->SetContext(this);
	node_state._father_name = father_name;
	_states[name] = node_state;
	return state;
}

// ���µ�ǰ״̬
void Context::Update()
{
	_cur_node_state._state->update();
}

// ͬ���¼�
// ����һ���¼����ṩ��root״̬�͵�ǰ״̬����
// �����ǰ״̬����״̬���򻹻����״̬����
void Context::SendEvent(EventData event_data)
{
	RecursiveSend(_cur_node_state, event_data);
}

std::string Context::GetCurStateName()
{
	return _cur_name;
}

// �ݹ�send
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
		// ֹͣ��һ��״̬
		_cur_node_state._state->stop();

		// ��ʼ����һ��״̬
		_cur_node_state = iter_map->second;
		_cur_name = iter_map->first;
		_cur_node_state._state->start();
	}
}

void Context::ButtonCallback(const agv_msg::Button::ConstPtr &msg)
{
	ROS_INFO("OK");
	for (auto n : msg->button_type)
	{
		ROS_INFO("response %s request", msg->button_func);
		switch (n)
		{
		case next_:
		{			
			// EventData e = EventData((int)unlock_);
			// this->SendEvent(e);
			this->Update();
		}
		case goback_:
		{
			this->TransForState("Auto");
			EventData e = EventData((int)go_);
			goal_data start = {0.0,0.0,0.0};
			e.SetData(&start);
			this->SendEvent(e);
		}
		case stop_:
		{
			// EventData e = EventData((int)unlock_);
			// this->SendEvent(e);
			this->TransForState("Lock");
		}
		case unlock_:
		{
			// EventData e = EventData((int)unlock_);
			// this->SendEvent(e);
			this->TransForState("Telecontrol");
		}
		}
	}
}
