#include "../inverted_pendulum/inverted_pendulum.h"
#include "../controller/pid.h"
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
#define PARAM_angle 0.0 
// Initial speed of cart [m/s]
#define PARAM_v 0.0
// Initial position cart [m]
#define PARAM_x 5.0

// Duration of a simulation step [s]
#define PARAM_DT 0.001
// Duration of simulation [s]
#define PARAM_D 20

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
	pendulum_state_t state_initial = {PARAM_x, PARAM_v, PARAM_angle, 0.0};
	InvertedPendulum pendulum = InvertedPendulum(PARAM_m, PARAM_M, PARAM_I, PARAM_l, 0.0, state_initial);
	state_sequence_t states;

	PIDController pid_ctrl_angle(PARAM_KP_ANGLE, PARAM_KI_ANGLE, PARAM_KD_ANGLE);
	PIDController pid_ctrl_x(PARAM_KP_X, PARAM_KI_X, PARAM_KD_X);
	PIDController pid_ctrl_v(PARAM_KP_V, PARAM_KI_V, PARAM_KD_V);
	
	double t = 0.0;
	double phi_setpoint = 0.0;
	
	while (t < PARAM_D) {
		// Simulate pendulum until next sampling time.
		// Add simulated states to sequence of states.
		double dsim = t+PARAM_TSAMP;
		pendulum.simulate(dsim, PARAM_DT, states);
		t = states.back().first; // current time of simulation is time of last recorded state

		// Drive cart towards position setpoint by controlling the speed of the cart.
		double x = states.back().second[0];
		double v_setpoint = -pid_ctrl_x.control(PARAM_SETPOINT_X, x, t);
                // Clamp v to +- PARAM_V_CLAMP.
		if (v_setpoint >  PARAM_V_CLAMP)
			v_setpoint = PARAM_V_CLAMP;
		else if (v_setpoint < -PARAM_V_CLAMP)
			v_setpoint = -PARAM_V_CLAMP;
//		std::cout << "V set: " << v_setpoint << std::endl;
		
		// To adjust the speed, we need to accelarate the cart.
		// We indirectly control the cart acceleration by adjusting the angle setpoint of the pole:
		// - If the pole leans to the left (positive angle setpoint), the cart accelerates to the left.
		// - If the pole leans to the right (negative angle setpoint), the cart accelerates to the right.
		// - If the pole angle is zero, acceleration is zero.
		double v = states.back().second[1];
		double phi_setpoint = pid_ctrl_v.control(v_setpoint, v, t);
                // Clamp phi to +- PARAM_PHI_CLAMP.
		if (phi_setpoint >  PARAM_PHI_CLAMP)
			phi_setpoint = PARAM_PHI_CLAMP;
		else if (phi_setpoint < -PARAM_PHI_CLAMP)
			phi_setpoint = -PARAM_PHI_CLAMP;
//		std::cout << "Phi set: " << phi_setpoint << std::endl;
		
		// Get pendulum angle, call angle controller, and set force to cart.
		double phi = states.back().second[2];
		double u = -pid_ctrl_angle.control(phi_setpoint, phi, t);
		pendulum.set_force(u);
	}
	
	print_states_csv(states);
	
	return 0;
}
