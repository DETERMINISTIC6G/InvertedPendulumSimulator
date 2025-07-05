/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
 * SPDX-FileContributor: Elena Mostovaya (st169601@stud.uni-stuttgart.de)
 */
 
#ifndef EVENT_RECEIVER_H
#define EVENT_RECEIVER_H

#include "event.h"
#include <functional>
#include <string>

class EventReceiver
{

      public:
        std::function<void(const Event &)> action;

      public:
        EventReceiver() = default;
        virtual ~EventReceiver() = default;
};

#endif // EVENT_RECEIVER_H
