#ifndef EVENT_RECEIVER_H
#define EVENT_RECEIVER_H


#include <string>
#include "event.h"
#include <functional>

class EventReceiver
{

public:
    std::function<void(const Event&)> action;


public:
    EventReceiver() = default;
    virtual ~EventReceiver() = default;
};

#endif // EVENT_RECEIVER_H
