/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
 * SPDX-FileContributor: Elena Mostovaya (st169601@stud.uni-stuttgart.de)
 */
 
#include "../controller/lqr.h"
#include "../controller/pid.h"
#include "../events/event.h"
#include "../events/event_queue.h"
#include "../events/event_receiver.h"
#include "../inverted_pendulum/inverted_pendulum.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <unistd.h>

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
// Initial position cart [m]
#define PARAM_x 0.0

// Duration of a simulation step [s]
#define PARAM_DT 0.0001

// PID controller parameters
#define PARAM_SETPOINT 0.0
#define PARAM_KP 10.0
#define PARAM_KI 1.0
#define PARAM_KD 1.0

// LQR gain matrix
#define LQR_K_ANGLE                                                                                                    \
        {                                                                                                              \
                -1.0000000000001679, -2.7126628569811633, 42.94618303488281, 5.411763498735041                         \
        }

#define MAX_STR_LEN 1024

char pathInputCSVFile[MAX_STR_LEN];
char pathOutputCSVFile[MAX_STR_LEN];

int simNumber = 0;

/**
 * Print usage information for the command line arguments.
 */
void usage(const char *progname)
{
        fprintf(stderr,
                "Usage: %s -i <input.csv> -o <output.csv> -n <sim_number>\n"
                "Options:\n"
                "  -i <input.csv>     Path to the input CSV file.\n"
                "  -o <output.csv>    Path to the output CSV file.\n"
                "  -n <sim_number>    Simulation number (integer). Select a simulation 1 (PID) or 2 (LQR).\n",
                progname);
}

/**
 * Parse command line arguments as passed to main() and store them in
 * global variables.
 */
int parse_cmdline_args(int argc, char *argv[])
{
        int opt;

        memset(pathInputCSVFile, 0, MAX_STR_LEN);

        while ((opt = getopt(argc, argv, "i:o:n:")) != -1) {
                switch (opt) {
                case 'i':
                        strncpy(pathInputCSVFile, optarg, MAX_STR_LEN - 1);
                        break;
                case 'o':
                        strncpy(pathOutputCSVFile, optarg, MAX_STR_LEN - 1);
                        break;
                case 'n':
                        simNumber = atoi(optarg);
                        break;
                case ':':
                case '?':
                default:
                        return -1;
                }
        }

        if (strlen(pathInputCSVFile) == 0 || strlen(pathOutputCSVFile) == 0)
                return -1;

        return 0;
}

void print_states_csv_to_file(const state_sequence_t &states, const std::string &filename)
{
        std::ofstream out(filename);
        if (!out.is_open()) {
                perror("Could not open file");
                return;
        }
        out << "t,x,v,phi,omega" << std::endl;
        for (const time_state_t &ts : states) {
                out << ts.first << "," << ts.second[0] << "," << ts.second[1] << ","
                    << ts.second[2] //* (180.0 / M_PI) //Convert to degrees
                    << "," << ts.second[3] << std::endl;
        }
        out.close();
}

void print_states_csv(const state_sequence_t &states)
{
        std::cout << "# t,x,v,phi,omega" << std::endl;
        for (const time_state_t &ts : states) {
                std::cout << ts.first << "," << ts.second[0] << "," << ts.second[1] << "," << ts.second[2] << ","
                          << ts.second[3] << std::endl;
        }
}

void simulate_pid(double untilTime)
{
        /*
         * Initial pendulum state vector:
         *
         * [  x  ]
         * [  v  ]
         * [ phi ]
         * [omega]
         */
        pendulum_state_t state_initial = {PARAM_x, PARAM_v, PARAM_angle, 0.0};
        InvertedPendulum pendulum = InvertedPendulum(PARAM_m, PARAM_M, PARAM_I, PARAM_l, 0.0, state_initial);
        state_sequence_t states;

        PIDController pidCtrl(PARAM_KP, PARAM_KI, PARAM_KD);

        // Initialize the queue with events from a CSV file.
        // Schedule periodic update events.
        EventQueue eventQueue = EventQueue(pathInputCSVFile, PARAM_DT);

        vector<double> u_vec = {};
        u_vec.push_back(0.0);

        unsigned long nextSendSeqNumber = 0;
        unsigned long currentRcvSeqNumber = 0;

        // If an update from the controller is available, update system input.
        // If no update is available, keep the old value of the system input.
        // Maybe, we have missed some updates, but we can always read the latest update.
        pendulum.action = [&pendulum, &u_vec, &states, &nextSendSeqNumber, &currentRcvSeqNumber](const Event &e) {
                if (e.type == Event::Type::UPDATE) {
                        printf("PLANT: update at %f, event %lu, f= %f\n", e.time, e.eventId, pendulum.get_force());
                        pendulum.simulate(PARAM_DT, states);
                } else if (e.type == Event::Type::RECEIVE) {
                        printf("PLANT: receive at %f, event %lu, SeqNr %lu\n", e.time, e.eventId, e.pktNr);
                        if (e.pktNr >= currentRcvSeqNumber) {
                                pendulum.set_force(u_vec[e.pktNr]);
                                currentRcvSeqNumber = e.pktNr;
                        }
                } else if (e.type == Event::Type::SEND) {
                        ++nextSendSeqNumber;
                        printf("PLANT: send at %f, event %lu. next seqNr: %lu\n", e.time, e.eventId, nextSendSeqNumber);
                }
        };

        pidCtrl.action = [&pendulum, &pidCtrl, &nextSendSeqNumber, &states, &u_vec](const Event &e) {
                if (e.type == Event::Type::SEND) {
                        // Get pendulum angle, call controller, and set force to cart.
                        if (!states.empty() && e.pktNr == nextSendSeqNumber - 1) {
                                double phi = states.back().second[2];
                                double t = states.back().first;
                                u_vec.push_back(-pidCtrl.control(PARAM_SETPOINT, phi, t));
                                printf("CONTROLLER: compute next U = %f at %f, event %lu, pctNr: %lu\n", u_vec.back(),
                                       e.time, e.eventId, e.pktNr);
                        } else if (!states.empty()) {
                                printf("CONTROLLER: out-of-order packet, no update at %f, event %lu\n", e.time,
                                       e.eventId);
                        }
                }
        };

        // Make sure the order is correct
        eventQueue.addReceiver(pendulum.action);
        eventQueue.addReceiver(pidCtrl.action);

        eventQueue.run(untilTime);

        print_states_csv_to_file(states, pathOutputCSVFile);
}

void simulate_lqr(double untilTime)
{
        /*
         * Initial pendulum state vector:
         *
         * [  x  ]
         * [  v  ]
         * [ phi ]
         * [omega]
         */
        pendulum_state_t state_initial = {PARAM_x, PARAM_v, PARAM_angle, 0.0};
        InvertedPendulum pendulum = InvertedPendulum(PARAM_m, PARAM_M, PARAM_I, PARAM_l, 0.0, state_initial);
        state_sequence_t states;

        LQRegulator lqr(LQR_K_ANGLE);

        // Initialize the queue with events from a CSV file.
        // Schedule periodic update events.
        EventQueue eventQueue = EventQueue(pathInputCSVFile, PARAM_DT);

        vector<double> u_vec = {};
        u_vec.push_back(0.0);

        unsigned long nextSendSeqNumber = 0;
        unsigned long currentRcvSeqNumber = 0;

        // If an update from the controller is available, update system input.
        // If no update is available, keep the old value of the system input.
        // Maybe, we have missed some updates, but we can always read the latest update.
        pendulum.action = [&pendulum, &u_vec, &states, &nextSendSeqNumber, &currentRcvSeqNumber](const Event &e) {
                if (e.type == Event::Type::UPDATE) {
                        printf("PLANT: update at %f, event %lu, f= %f\n", e.time, e.eventId, pendulum.get_force());
                        pendulum.simulate(PARAM_DT, states);
                } else if (e.type == Event::Type::RECEIVE) {
                        printf("PLANT: receive at %f, event %lu, seqNr %lu\n", e.time, e.eventId, e.pktNr);
                        if (e.pktNr >= currentRcvSeqNumber) {
                                pendulum.set_force(u_vec[e.pktNr]);
                                currentRcvSeqNumber = e.pktNr;
                        }
                } else if (e.type == Event::Type::SEND) {
                        ++nextSendSeqNumber;
                        printf("PLANT: send at %f, event %lu. next seqNr: %lu\n", e.time, e.eventId, nextSendSeqNumber);
                }
        };

        lqr.action = [&pendulum, &lqr, &nextSendSeqNumber, &states, &u_vec](const Event &e) {
                if (e.type == Event::Type::SEND) {
                        // Get pendulum angle, call controller, and set force to cart.
                        if (!states.empty() && e.pktNr == nextSendSeqNumber - 1) {
                                u_vec.push_back(lqr.control(states.back().second));
                                printf("CONTROLLER: compute next U = %f at %f, event %lu, pctNr: %lu\n", u_vec.back(),
                                       e.time, e.eventId, e.pktNr);
                        } else if (!states.empty()) {
                                printf("CONTROLLER: out-of-order packet, no update at %f, event %lu\n", e.time,
                                       e.eventId);
                        }
                }
        };

        // Make sure the order is correct
        eventQueue.addReceiver(pendulum.action);
        eventQueue.addReceiver(lqr.action);

        eventQueue.run(untilTime);

        print_states_csv_to_file(states, pathOutputCSVFile);
}

int main(int argc, char *argv[])
{
        if (parse_cmdline_args(argc, argv) == -1) {
                usage(argv[0]);
                exit(1);
        }

        switch (simNumber) {
        case 1:
                simulate_pid(60.0);
                break;
        case 2:
                simulate_lqr(60.0);
                break;
        default:
                std::cout << "Select a simulation 1 (PID) or 2 (LQR)." << std::endl;
                return -1;
        }

        std::cout << "Simulation finished." << std::endl;

        return 0;
}
