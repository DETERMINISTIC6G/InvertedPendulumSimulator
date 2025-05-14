#ifndef EVENT_H
#define EVENT_H

#include <string>
#include <functional>

// Event structure to hold the time and action
struct Event
{
    unsigned long eventId;
    unsigned long pktNr;
    double time;
    enum class Type
    {
        SEND,
        RECEIVE,
        UPDATE
    } type;

    std::function<void(Event &)> action;

    bool operator<(const Event &other) const
    {
        return time > other.time;
    }
};

#endif // EVENT_H