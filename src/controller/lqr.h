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

	double control(const pendulum_state_t state,  double pos);
	
private:
	const pendulum_state_t K;
};

#endif
