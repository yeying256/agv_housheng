#include <hfsm/Context.h>
#include <hfsm/State.h>
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

void Context::ButtonCallback(const agv_msg::Button::ConstPtr &msg)
{
	for (int n = 0; n < msg->button_type.size(); n++)
	{
		ROS_INFO("Current State is %s", this->GetCurStateName().c_str());
		ROS_INFO("response %s request", msg->button_func[n].c_str());
		switch (msg->button_type[n])
		{
		case next_:
		{
			// EventData e = EventData((int)unlock_);
			// this->SendEvent(e);
			this->Update();
			break;
		}
		case goback_:
		{
			this->TransForState("Auto");
			EventData e = EventData((int)go_);
			goal_data start = {0.0, 0.0, 1.0};
			e.SetData(&start);
			this->SendEvent(e);
			break;
		}
		case stop_:
		{
			// EventData e = EventData((int)unlock_);
			// this->SendEvent(e);
			this->TransForState("Lock");
			break;
		}
		case unlock_:
		{
			// EventData e = EventData((int)unlock_);
			// this->SendEvent(e);
			this->TransForState("Telecontrol");
			break;
		}
		}
	}
}
