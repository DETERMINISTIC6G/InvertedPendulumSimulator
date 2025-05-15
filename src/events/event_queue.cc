#include "event_queue.h"

void EventQueue::run(double untilTime)
{
    scheduleAt(0, step, untilTime); // Schedule the first cyclic UPDATE event
    while (!events.empty() && events.top().time <= untilTime)
    {
        Event next = events.top();
        events.pop();
        // printf("%d at %f , event %lu \n", next.type, next.time, next.eventId);
        next.action(next);
        notifyReceivers(next);
    }
}

bool EventQueue::empty() const
{
    return events.empty();
}

double EventQueue::nextTime() const
{
    return events.top().time;
}

void EventQueue::addReceiver(std::function<void(const Event &)> cb)
{
    callbacks.push_back(cb);
}

void EventQueue::schedule(unsigned long pktNr, double time, Event::Type type, std::function<void(Event &)> action)
{
    events.push({nextEventId++, pktNr, time, type, action});
}

void EventQueue::scheduleAt(double startTime, double step, double untilTime)
{
    schedule(0, startTime, Event::Type::UPDATE,
             [=](Event &evt)
             {
                 double nextTime = evt.time + step;
                 if (nextTime <= untilTime)
                 {
                     schedule(0, nextTime, Event::Type::UPDATE, evt.action); // Re-schedule
                     ;                                                       // printf("UPDATE at %f , event %lu \n", evt.time, evt.eventId);
                 }
             });
}

void EventQueue::notifyReceivers(Event &event)
{
    for (auto &cb : callbacks)
    {
        cb(event);
    }
}