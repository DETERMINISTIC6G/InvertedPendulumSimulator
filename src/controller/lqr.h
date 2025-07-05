/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
 * SPDX-FileContributor: Elena Mostovaya (st169601@stud.uni-stuttgart.de)
 */
 
#ifndef LQR_H
#define LQR_H

#include "../inverted_pendulum/inverted_pendulum.h"

class LQRegulator : public EventReceiver
{
      public:
        /**
         * @param K gain matrix
         */
        LQRegulator(const pendulum_state_t &K);

        /**
         * Get control output.
         *
         * @param state state
         * @return controller output u
         */
        double control(const pendulum_state_t state);

        double control(const pendulum_state_t state, double pos);

      private:
        const pendulum_state_t K;
};

#endif
