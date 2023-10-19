#ifndef _STATE_H_
#define _STATE_H_
#include "Event.h"
#include <functional>
#include <string>

class Context;

class State
{
public:
	virtual void start() {}

	virtual void update() {}

	virtual void stop() {}

	void set_event_func(std::function<EventDeal(EventData &)> func);

	EventDeal RunEventFunc(EventData &event_data);

	void SetContext(Context *context);

	void TransState(std::string name);

private:
	std::function<EventDeal(EventData &)> _event_func;

protected:
	Context *_context;
};

#endif
