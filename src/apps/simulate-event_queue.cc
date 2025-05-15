#include "../inverted_pendulum/inverted_pendulum.h"
#include "../controller/pid.h"
#include "../events/event.h"
#include "../events/event_receiver.h"
#include "../events/event_queue.h"
#include "../controller/lqr.h"

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
#define PARAM_angle 0.349 //0.349 
// Initial speed of cart [m/s]
#define PARAM_v 0.0
// Initial position cart [m]
#define PARAM_x 5.0

// Duration of a simulation step [s]
#define PARAM_DT 0.001
// Duration of simulation [s]
#define PARAM_D 10

// PID controller parameters
#define PARAM_SETPOINT 0.0
#define PARAM_KP 10.0
#define PARAM_KI 1.0
#define PARAM_KD 1.0

// Angle PID controller parameters
#define PARAM_KP_ANGLE 10.0
#define PARAM_KI_ANGLE 0.0
#define PARAM_KD_ANGLE 1

// Position PID controller parameters
#define PARAM_KP_X 1.0
#define PARAM_KI_X 0.0
#define PARAM_KD_X 0.1

// Velocity PID controller parameters
#define PARAM_KP_V 0.06
#define PARAM_KI_V 0.0
#define PARAM_KD_V 0.0

// Setpoint of the position
#define PARAM_SETPOINT_X 0.0

// Clamp phi setpoint to +- 20 degree 
#define PARAM_PHI_CLAMP 0.349

// Clamp v setpoint to +- 1 [m/s] 
#define PARAM_V_CLAMP 1.0

// Sampling period [s]
#define PARAM_TSAMP 0.01

// LQR gain matrix
#define LQR_K {-3.162277660168483, -6.105688949485788, 49.16351188321586, 7.204143097154165}


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
            << "," << ts.second[2] //* (180.0 / M_PI) //Convert to degrees
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
	pendulum_state_t state_initial = {0.0, PARAM_v, PARAM_angle, 0.0};
	InvertedPendulum pendulum = InvertedPendulum(PARAM_m, PARAM_M, PARAM_I, PARAM_l, 0.0, state_initial);
	state_sequence_t states;

	PIDController pidCtrl(PARAM_KP, PARAM_KI, PARAM_KD);

    // Initialize the queue with events from a CSV file.
    // Schedule periodic update events.
    std::string pathCSVFile = "events.csv";
    EventQueue eventQueue = EventQueue(pathCSVFile);

    vector<double> tmp_u_vec = {};
    unsigned long nextSendSeqNumber = 0;  
    unsigned long currentRcvSeqNumber = 0;
   
    // If an update from the controller is available, update system input.
    // If no update is available, keep the old value of the system input.
    // Maybe, we have missed some updates, but we can always read the latest update.
    pendulum.action = [&pendulum, &tmp_u_vec, &states,
                        &nextSendSeqNumber, &currentRcvSeqNumber](const Event &e) {
        if (e.type == Event::Type::UPDATE) {
            printf("PLANT: update at %f, event %lu, f= %f\n", e.time, e.eventId, pendulum.get_force());
            pendulum.simulate(PARAM_DT, states);          
           
        }else if (e.type == Event::Type::RECEIVE) {
            printf("PLANT: receive at %f, event %lu\n", e.time, e.eventId);
            if (e.pktNr > currentRcvSeqNumber) {
                pendulum.set_force(tmp_u_vec[e.pktNr]);
                currentRcvSeqNumber = e.pktNr;
            }
        }else if (e.type == Event::Type::SEND) {
            ++nextSendSeqNumber;
            printf("PLANT: send at %f, event %lu. next seqNr: %lu\n", e.time, e.eventId, nextSendSeqNumber);
        }
    };

    pidCtrl.action = [&pendulum, &pidCtrl, 
                                    &nextSendSeqNumber, &states, &tmp_u_vec](const Event &e) {
        if (e.type == Event::Type::SEND) {            
			// Get pendulum angle, call controller, and set force to cart.
		    if (!states.empty() && e.pktNr == nextSendSeqNumber - 1) {
                double phi = states.back().second[2];
                double t = states.back().first;
                tmp_u_vec.push_back(-pidCtrl.control(PARAM_SETPOINT, phi, t)); 
                printf("CONTROLLER: compute next U = %f at %f, event %lu, pctNr: %lu\n", 
                            tmp_u_vec.back(), e.time, e.eventId, e.pktNr);   
            }else {
                printf("CONTROLLER: out-of-order packet, no update at %f, event %lu\n", e.time, e.eventId);
                tmp_u_vec.push_back(pendulum.get_force());
            }
            
        }
    };
  
    // Make sure the order is correct
    eventQueue.addReceiver(pendulum.action);
    eventQueue.addReceiver(pidCtrl.action);
  
    eventQueue.run(untilTime);

	//print_states_csv(states);
	print_states_csv_to_file(states, "states-eventqueue-one-pid.csv");
}




void simulate_position_angle(double untilTime)
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

	PIDController pid_ctrl_angle(PARAM_KP_ANGLE, PARAM_KI_ANGLE, PARAM_KD_ANGLE);
	PIDController pid_ctrl_x(PARAM_KP_X, PARAM_KI_X, PARAM_KD_X);
	PIDController pid_ctrl_v(PARAM_KP_V, PARAM_KI_V, PARAM_KD_V);

    // Initialize the queue with events from a CSV file.
    // Schedule periodic update events.
    std::string pathCSVFile = "events-2.csv";
    EventQueue eventQueue = EventQueue(pathCSVFile);

    vector<double> tmp_u_vec = {};
    unsigned long nextSendSeqNumber = 0;  
    unsigned long currentRcvSeqNumber = 0;
   
    // If an update from the controller is available, update system input.
    // If no update is available, keep the old value of the system input.
    // Maybe, we have missed some updates, but we can always read the latest update.
    pendulum.action = [&pendulum, &tmp_u_vec, &states,
                        &nextSendSeqNumber, &currentRcvSeqNumber](const Event &e) {
        if (e.type == Event::Type::UPDATE) {
            printf("PLANT: update at %f, event %lu, f= %f\n", e.time, e.eventId, pendulum.get_force());
            pendulum.simulate(PARAM_DT, states);          
           
        }else if (e.type == Event::Type::RECEIVE) {
            printf("PLANT: receive at %f, event %lu\n", e.time, e.eventId);
            if (e.pktNr > currentRcvSeqNumber) {
                pendulum.set_force(tmp_u_vec[e.pktNr]);
                currentRcvSeqNumber = e.pktNr;
            }
        }else if (e.type == Event::Type::SEND) {
            ++nextSendSeqNumber;
            printf("PLANT: send at %f, event %lu. next seqNr: %lu\n", e.time, e.eventId, nextSendSeqNumber);
        }
    };

    pid_ctrl_angle.action = [&pendulum, &pid_ctrl_angle, &pid_ctrl_x, &pid_ctrl_v,
                                &nextSendSeqNumber, &states, &tmp_u_vec](const Event &e) {
        if (e.type == Event::Type::SEND) {            
			// Get pendulum angle, call controller, and set force to cart.
                   
		    if (!states.empty() && e.pktNr == nextSendSeqNumber - 1) {
                // Drive cart towards position setpoint by controlling the speed of the cart.
                double t = states.back().first;   
                double x = states.back().second[0];
                double v_setpoint = -pid_ctrl_x.control(PARAM_SETPOINT_X, x, t);
                // Clamp v to +- PARAM_V_CLAMP.
                if (v_setpoint >  PARAM_V_CLAMP)
                    v_setpoint = PARAM_V_CLAMP;
                else if (v_setpoint < -PARAM_V_CLAMP)
                    v_setpoint = -PARAM_V_CLAMP;
                
                double v = states.back().second[1];
                double phi_setpoint = pid_ctrl_v.control(v_setpoint, v, t);
                // Clamp phi to +- PARAM_PHI_CLAMP.
                if (phi_setpoint >  PARAM_PHI_CLAMP)
                    phi_setpoint = PARAM_PHI_CLAMP;
                else if (phi_setpoint < -PARAM_PHI_CLAMP)
                    phi_setpoint = -PARAM_PHI_CLAMP;
                               
                double phi = states.back().second[2];
                tmp_u_vec.push_back(-pid_ctrl_angle.control(phi_setpoint, phi, t));
                printf("CONTROLLER: compute next U = %f at %f, event %lu, pctNr: %lu\n", 
                    tmp_u_vec.back(), e.time, e.eventId, e.pktNr);                                    
            }else {
                printf("CONTROLLER: Out-of-order packet, no update at %f, event %lu\n", e.time, e.eventId);
                tmp_u_vec.push_back(pendulum.get_force());
            }          
        }
    };
  
    // Make sure the order is correct
    eventQueue.addReceiver(pendulum.action);
    eventQueue.addReceiver(pid_ctrl_angle.action);
  
    eventQueue.run(untilTime);

	//print_states_csv(states);
	print_states_csv_to_file(states, "states-eventqueue-three-pids.csv");
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

	LQRegulator lqr(LQR_K);

    // Initialize the queue with events from a CSV file.
    // Schedule periodic update events.
    std::string pathCSVFile = "events-2.csv";
    EventQueue eventQueue = EventQueue(pathCSVFile);

    vector<double> tmp_u_vec = {};
    unsigned long nextSendSeqNumber = 0;  
    unsigned long currentRcvSeqNumber = 0;
   
    // If an update from the controller is available, update system input.
    // If no update is available, keep the old value of the system input.
    // Maybe, we have missed some updates, but we can always read the latest update.
    pendulum.action = [&pendulum, &tmp_u_vec, &states,
                        &nextSendSeqNumber, &currentRcvSeqNumber](const Event &e) {
        if (e.type == Event::Type::UPDATE) {
            printf("PLANT: update at %f, event %lu, f= %f\n", e.time, e.eventId, pendulum.get_force());
            pendulum.simulate(PARAM_DT, states);          
           
        }else if (e.type == Event::Type::RECEIVE) {
            printf("PLANT: receive at %f, event %lu\n", e.time, e.eventId);
            if (e.pktNr > currentRcvSeqNumber) {
                pendulum.set_force(tmp_u_vec[e.pktNr]);
                currentRcvSeqNumber = e.pktNr;
            }
        }else if (e.type == Event::Type::SEND) {
            ++nextSendSeqNumber;
            printf("PLANT: send at %f, event %lu. next seqNr: %lu\n", e.time, e.eventId, nextSendSeqNumber);
        }
    };

    lqr.action = [&pendulum, &lqr,
                                &nextSendSeqNumber, &states, &tmp_u_vec](const Event &e) {
        if (e.type == Event::Type::SEND) {            
			        
		    if (!states.empty() && e.pktNr == nextSendSeqNumber - 1) {
                
                tmp_u_vec.push_back(lqr.control(states.back().second));
                printf("CONTROLLER: compute next U = %f at %f, event %lu, pctNr: %lu\n", 
                                    tmp_u_vec.back(), e.time, e.eventId, e.pktNr);                                    
            }else {
                printf("CONTROLLER: out-of-order packet, no update at %f, event %lu\n", e.time, e.eventId);
                tmp_u_vec.push_back(pendulum.get_force());
            }          
        }
    };
  
    // Make sure the order is correct
    eventQueue.addReceiver(pendulum.action);
    eventQueue.addReceiver(lqr.action);
  
    eventQueue.run(untilTime);

	//print_states_csv(states);
	print_states_csv_to_file(states, "states-eventqueue-lqr.csv");
}




int main(int argc, char *argv[])
{
	//simulate_pid(20.0);
    //simulate_position_angle(60.0);
    simulate_lqr(60.0);
    std::cout << "Simulation finished." << std::endl;

	return 0;
}
