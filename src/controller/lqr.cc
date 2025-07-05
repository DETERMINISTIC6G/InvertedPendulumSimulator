/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
 * SPDX-FileContributor: Elena Mostovaya (st169601@stud.uni-stuttgart.de)
 */
 
#include "lqr.h"
#include <cmath>

LQRegulator::LQRegulator(const pendulum_state_t &K) : K(K)
{
}

double LQRegulator::control(const pendulum_state_t state)
{
        double u = -(K[0] * state[0] + K[1] * state[1] + K[2] * state[2] + K[3] * state[3]);

        return u;
}

double LQRegulator::control(const pendulum_state_t state, double pos)
{
        double u = -(K[0] * state[0] - K[0] * pos + K[1] * state[1] + K[2] * state[2] + K[3] * state[3]);

        return u;
}
