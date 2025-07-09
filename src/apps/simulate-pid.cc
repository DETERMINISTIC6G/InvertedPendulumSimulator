/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
 * SPDX-FileContributor: Elena Mostovaya (st169601@stud.uni-stuttgart.de)
 */
 
#include "../controller/pid.h"
#include "../inverted_pendulum/inverted_pendulum.h"
#include <iostream>

// Mass of pendulum [kg]
#define PARAM_m 0.2
// Mass of cart [kg]
#define PARAM_M 0.5
// Moment of Inertia [kg*m^2]
#define PARAM_I 0.006
// Length of pendulum to center of mass [m]
#define PARAM_l 0.3
// Initial angle of pendulum [rad]
#define PARAM_angle 0.349
// Initial speed of cart [m/s]
#define PARAM_v 0.0

// Duration of a simulation step [s]
#define PARAM_DT 0.001
// Duration of simulation [s]
#define PARAM_D 10

// PID controller parameters
#define PARAM_SETPOINT 0.0
#define PARAM_KP 10.0
#define PARAM_KI 1.0
#define PARAM_KD 1.0

// Sampling period [s]
#define PARAM_TSAMP 0.01

void print_states_csv(const state_sequence_t &states)
{
        std::cout << "# t,x,v,phi,omega" << std::endl;
        for (const time_state_t &ts : states) {
                std::cout << ts.first << "," << ts.second[0] << "," << ts.second[1] << "," << ts.second[2] << ","
                          << ts.second[3] << std::endl;
        }
}

int main(int argc, char *argv[])
{
        /*
         * Initial pendulum state vector:
         *
         * [  x  ]
         * [  v  ]
         * [ phi ]
         * [omega]
         */
        pendulum_state_t state_initial = {0.0, PARAM_v, PARAM_angle, 0.0};
        InvertedPendulum pendulum = InvertedPendulum(PARAM_m, PARAM_M, PARAM_I, PARAM_l, 0.0, state_initial);
        state_sequence_t states;

        PIDController pidCtrl(PARAM_KP, PARAM_KI, PARAM_KD);

        double t = 0.0;

        while (t < PARAM_D) {
                // Simulate pendulum until next sampling time.
                // Add simulated states to sequence of states.
                pendulum.simulate(PARAM_TSAMP, PARAM_DT, states);
                t = states.back().first; // current time of simulation is time of last recorded state
                // Get pendulum angle, call controller, and set force to cart.
                double phi = states.back().second[2];
                double u = -pidCtrl.control(PARAM_SETPOINT, phi, t);
                pendulum.set_force(u);
        }

        print_states_csv(states);

        return 0;
}
