#ifndef _EVENT_H_
#define _EVENT_H_

enum EventDeal
{
    tail = 0, // 事件结束处理
    keep_on   // 事件继续传递
};

// 事件数据
class EventData
{
public:
    EventData(int event_type)
    {
        _event_type = event_type;
        _data = nullptr;
    }

    template <typename T>
    void SetData(T *t)
    {
        _data = t;
    }

    template <typename T>
    T *GetData()
    {
        return (T *)_data;
    }

    int _event_type;

private:
    void *_data;
};
#endif
