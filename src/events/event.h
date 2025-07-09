/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
 * SPDX-FileContributor: Elena Mostovaya (st169601@stud.uni-stuttgart.de)
 */
 
#ifndef EVENT_H
#define EVENT_H

#include <functional>
#include <string>

// Event structure to hold the time and action
struct Event {
        unsigned long eventId;
        unsigned long pktNr;
        double time;
        enum class Type {
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