#include "../inverted_pendulum/inverted_pendulum.h"
#include "../controller/pid.h"
#include "../events/event.h"
#include "../events/event_receiver.h"
#include "../events/event_queue.h"

#include <iostream>
#include <fstream>
#include <cmath> 

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

void print_states_csv_to_file(const state_sequence_t &states, const std::string &filename)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        perror("Could not open file");
		return;
    }
    out << "t,x,v,phi,omega" << std::endl;
    for (const time_state_t &ts : states) {
        out << ts.first
            << "," << ts.second[0]
            << "," << ts.second[1]
            << "," << ts.second[2] * (180.0 / M_PI)
            << "," << ts.second[3]
            << std::endl;
    }
    out.close();
}

void print_states_csv(const state_sequence_t &states)
{
	std::cout << "# t,x,v,phi,omega" << std::endl;
	for (const time_state_t &ts: states) {
		std::cout << ts.first
			  << "," << ts.second[0]
			  << "," << ts.second[1]
			  << "," << ts.second[2]
			  << ","  << ts.second[3]
			  << std::endl;
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

    // Initialize the queue with events from a CSV file.
    // Schedule periodic update events.
    std::string pathCSVFile = "events.csv";
    EventQueue eventQueue = EventQueue(pathCSVFile);

    double tmp_u = 0.0;
    unsigned long nextSeqNumber = 0;  
    unsigned long currentSeqNumber = 0;
   
    // If an update from the controller is available, update system input.
    // If no update is available, keep the old value of the system input.
    // Maybe, we have missed some updates, but we can always read the latest update.
    pendulum.action = [&pendulum, &tmp_u, &nextSeqNumber, &states](const Event &e) {
        if (e.type == Event::Type::UPDATE) {
            printf("PLANT: update at %f, event %lu\n", e.time, e.eventId);
            pendulum.simulate(PARAM_DT, states);          
           
        }else if (e.type == Event::Type::RECEIVE) {
            printf("PLANT: receive at %f, event %lu\n", e.time, e.eventId);
            if (e.pktNr >= nextSeqNumber - 1) {
                pendulum.set_force(tmp_u);
                //currentSeqNumber = e.pktNr;
            }
        }else if (e.type == Event::Type::SEND) {
            nextSeqNumber++;
            printf("PLANT: send at %f, event %lu\n", e.time, e.eventId);
        }
    };

    pidCtrl.action = [&pidCtrl, &states, &tmp_u](const Event &e) {
        if (e.type == Event::Type::SEND) {            
			// Get pendulum angle, call controller, and set force to cart.
            double phi = PARAM_angle;
            double t = 0.0; 
		    if (!states.empty()) {
                phi = states.back().second[2];
                t = states.back().first;              
            }
		     tmp_u = -pidCtrl.control(PARAM_SETPOINT, phi, t);
		    
            
            printf("CONTROLLER: receive and compute new U = %f at %f, event %lu\n", tmp_u, e.time, e.eventId);
        }
    };

    
    eventQueue.addReceiver(pendulum.action);
    eventQueue.addReceiver(pidCtrl.action);
  
    double untilTime = 59.0;
    eventQueue.run(untilTime);

	//print_states_csv(states);
	print_states_csv_to_file(states, "states-eventqueue.csv");

	return 0;
}
