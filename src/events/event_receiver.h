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
    //virtual void handleEvent() = 0;
    virtual ~EventReceiver() = default;   
    //virtual void shutdown() = 0;
};

#endif // EVENT_RECEIVER_H
