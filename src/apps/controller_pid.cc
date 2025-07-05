/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
 */

#include <iostream>
#include <unistd.h>
#include <inttypes.h>

#include "inverted_pendulum.h"
#include "pid.h"
#include "../netutils/socket_utils.h"
#include "marshaling.h"

#define MAX_STR_LEN 1024
#define MAX_PKT_SIZE 65535

// Global configuration parameters.
char ctrl_service[MAX_STR_LEN];
int sock;

double kp = 30.0;
double ki = 0.0;
double kd = 0.0;

/**
 * Exit application with given exit status.
 * Clean up before exiting.
 *
 * @param exit_status exit status
 */
void die(int exit_status)
{
     exit(exit_status);
}

/**
 * Print usage information.
 */
void usage(const char *prog)
{
     fprintf(stderr, "Usage: %s \n"
             "-p PORT : destination service (service name or port number) \n"
	     "-P PVAL : P(roportional) value of PID controller \n"
	     "-I IVAL : I(ntegral) value of PID controller \n"
	     "-D DVAL : D(ifferential) value of PID controller \n"
             "\n", prog);
}

/**
 * Parse command line arguments as passed to main() and store them in
 * global variables.
 */
int parse_cmdline_args(int argc, char *argv[])
{
     int opt;
     bool is_p = false;
     bool is_i = false;
     bool is_d = false;
     
     memset(ctrl_service, 0, MAX_STR_LEN);

     while ( (opt = getopt(argc, argv, "p:P:I:D:")) != -1 ) {
	     switch(opt) {
	     case 'p' :
		     strncpy(ctrl_service, optarg, MAX_STR_LEN-1);
		     break;
	     case 'P' :
		     kp = atof(optarg);
		     is_p = true;
		     break;
	     case 'I' :
		     ki = atof(optarg);
		     is_i = true;
		     break;
	     case 'D' :
		     kd = atof(optarg);
		     is_d = true;
		     break;
	     case ':' :
	     case '?' :
	     default :
		     return -1;
	     }
     }
     
     if (strlen(ctrl_service) == 0 || !is_p || !is_i || !is_d)
          return -1;

     return 0;
}

int main(int argc, char *argv[])
{
	if (parse_cmdline_args(argc, argv) == -1) {
		usage(argv[0]);
		die(1);
	}	

	if (datagram_server_sockets(NULL, ctrl_service, AF_UNSPEC, 0, &sock, 1) != 1) {
		perror("Could not create socket");
		die(1);
	}
	
	// Set PID constants
	/*
	const double kp = 100.0;
	const double ki = 50.0;
	const double kd = 10.0;
	*/
	
	// Create a PID controller
	PID *ctrl_pid = new PID();
	ctrl_pid->Init(kp, ki, kd);
	
	while (true) {
		uint8_t data[MAX_PKT_SIZE];
		struct sockaddr_storage plant_addr;
		socklen_t plant_addr_len = sizeof(plant_addr);
		ssize_t data_len = recvfrom(sock, data, MAX_PKT_SIZE, 0,
					    (sockaddr *) &plant_addr, &plant_addr_len);
		if (data_len == -1) {
			perror("Could not receive message");
		}

		double angle;
		uint64_t t_usec;
		if (!demarshaling_state(data, data_len, &t_usec, &angle)) {
			fprintf(stderr, "Could not demarshal state.\n");
			continue;
		}

		//printf("State received: time = %" PRIu64 " us  angle = %f degree\n", t_usec, angle);
		
		double error = 0.0-angle;
		ctrl_pid->UpdateError(((float) t_usec)/1000000.0, error);
		double u = ctrl_pid->TotalError();

		data_len = marshaling_update(data, MAX_PKT_SIZE, t_usec, u);
		if (data_len == -1) {
			fprintf(stderr, "Could not marshal update.\n");
			continue;
		}

		ssize_t n_sent = sendto(sock, data, data_len, 0, (sockaddr *) &plant_addr, plant_addr_len);
		if (n_sent != data_len) {
			perror("Could not send update");
		}
	}
  
	return 0;
}
