#include "pid.h"
#include <cassert>

PIDController::PIDController(double kp, double ki, double kd) :
	kp(kp), ki(ki), kd(kd), eint(0.0), eprev(0.0), tprev(0.0)
{
}

double PIDController::control(double setpoint, double yt, double t)
{
	double e = yt-setpoint;

	double dt = t-tprev;
	assert(dt >= 0.0);
	
	eint += 0.5*(e+eprev)*dt;

	double ediff;
	if (dt > 0.0)
		ediff = (e-eprev)/dt;
	else
		ediff = 0.0;

	double u = kp*e + ki*eint + kd*ediff;
		
	eprev = e;
	tprev = t;

	return u;
}
