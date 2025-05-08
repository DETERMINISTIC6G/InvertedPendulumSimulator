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
#define PARAM_angle 0.087 
// Force onto cart [N]
#define PARAM_F 1.0
// Initial speed of cart [m/s]
#define PARAM_v 0.0

// Duration of a simulation step [s]
#define PARAM_DT 0.001
// Duration of simulation [s]
#define PARAM_D 10

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
	InvertedPendulum pendulum = InvertedPendulum(PARAM_m, PARAM_M, PARAM_I, PARAM_l, PARAM_F, state_initial);
	state_sequence_t states;

	pendulum.simulate(PARAM_D, PARAM_DT, states);

	print_states_csv(states);
	
	return 0;
}
