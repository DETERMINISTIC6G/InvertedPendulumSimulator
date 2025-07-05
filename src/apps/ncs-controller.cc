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
#include <string.h>
#include <sys/socket.h>

#include "../inverted_pendulum/inverted_pendulum.h"
#include "../controller/lqr.h"
#include "marshaling.h"

#define MAX_STR_LEN 1024
#define MAX_PKT_SIZE 65535

// LQR gain matrix
#define LQR_K {-3.162277660168483, -6.105688949485788, 49.16351188321586, 7.204143097154165}

// Global configuration parameters.
char ctrl_service[MAX_STR_LEN];
int sock;

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
             "-p PORT : service name or port number \n"
             "\n", prog);
}

/**
 * Parse command line arguments as passed to main() and store them in
 * global variables.
 */
int parse_cmdline_args(int argc, char *argv[])
{
     int opt;
     
     memset(ctrl_service, 0, MAX_STR_LEN);

     while ( (opt = getopt(argc, argv, "p:")) != -1 ) {
	     switch(opt) {
	     case 'p' :
		     strncpy(ctrl_service, optarg, MAX_STR_LEN-1);
		     break;
	     case ':' :
	     case '?' :
	     default :
		     return -1;
	     }
     }
     
     if (strlen(ctrl_service) == 0)
          return -1;

     return 0;
}

int main(int argc, char *argv[])
{
	if (parse_cmdline_args(argc, argv) == -1) {
		usage(argv[0]);
		die(1);
	}	

	// TODO: Create server socket for communication with plant.
	
	// Create LQR.
	LQRegulator lqr(LQR_K);

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
		double omega;
		double x;
		double v;
		uint64_t t_usec;
		if (!demarshaling_state(data, data_len, t_usec, angle, omega, x, v)) {
			fprintf(stderr, "Could not demarshal state.\n");
			continue;
		}
		//printf("State received: time = %" PRIu64 " us  angle = %f degree\n", t_usec, angle);
		pendulum_state_t state = {x, v, angle, omega};
		
		double u = lqr.control(state);

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
