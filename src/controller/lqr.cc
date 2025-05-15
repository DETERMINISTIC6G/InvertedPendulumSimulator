#include "lqr.h"

LQRegulator::LQRegulator(const pendulum_state_t &K) :
	K(K)
{
}

double LQRegulator::control(const pendulum_state_t state)
{
	// u = -K*state
	double u = -(K[0]*state[0] + K[1]*state[1] + K[2]*state[2] + K[3]*state[3]);

	return u;
}

