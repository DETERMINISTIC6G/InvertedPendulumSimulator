#ifndef PID_H
#define PID_H

#include "../events/event_receiver.h"

class PIDController : public EventReceiver
{
public:
	/**
	 * @param kp parameter P of PID controller
	 * @param ki parameter I of PID controller
	 * @param kd parameter D of PID controller
	 */
	PIDController(double kp, double ki, double kd);

	/**
	 * Get control output.
	 *
	 * @param setpoint setpoint
	 * @param yt measured value
	 * @param t current time
	 * @return controller output u
	 */
	double control(double setpoint, double yt, double t);
	
private:
	const double kp, ki, kd;

	double eint;
	double eprev;
	double tprev;
};

#endif
