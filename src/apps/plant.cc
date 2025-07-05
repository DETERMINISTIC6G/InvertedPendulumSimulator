/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
 */

/**
 * This file incoperates work covered by the following copyright and  
 * permission notice:
 *
 * Copyright (c) 2021 Jose Antonio Sanchez Leon
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <SFML/Graphics.hpp>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <atomic>
#include <mutex>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "inverted_pendulum.h"
#include "tools.h"
#include "../netutils/socket_utils.h"
#include "marshaling.h"

#define MAX_STR_LEN 1024
#define MAX_PKT_SIZE 65535

#define LOG_INTERVAL_USEC 10000

// Global configuration parameters.
char ctrl_host[MAX_STR_LEN];
char ctrl_service[MAX_STR_LEN];
uint64_t cycletime_usec = 0;

int sock = -1;

char log_file_path[MAX_STR_LEN];

pthread_t thread;
std::atomic_int update_ready(0);
std::mutex update_lock;
struct {
	double u;
} update;

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
             "-d HOST : destination host (name or IP address) \n"
             "-p PORT : destination service (service name or port number) \n"
             "-c CYCLETIME : cycle time in micro-seconds for sending datagrams \n"
	     "-f FILENAME : log file \n"
             "\n", prog);
}

/**
 * Parse command line arguments as passed to main() and store them in
 * global variables.
 */
int parse_cmdline_args(int argc, char *argv[])
{
     int opt;
     memset(ctrl_host, 0, MAX_STR_LEN);
     memset(ctrl_service, 0, MAX_STR_LEN);
     memset(log_file_path, 0, MAX_STR_LEN);
     bool isdef_cycletime = false;

     
     while ( (opt = getopt(argc, argv, "d:p:c:f:")) != -1 ) {
	     switch(opt) {
	     case 'd' :
		     strncpy(ctrl_host, optarg, MAX_STR_LEN-1);
		     break;
	     case 'p' :
		     strncpy(ctrl_service, optarg, MAX_STR_LEN-1);
		     break;
	     case 'c' :
		     cycletime_usec = strtoull(optarg, NULL, 10);
		     isdef_cycletime = true;
		     break;
	     case 'f' :
		     strncpy(log_file_path, optarg, MAX_STR_LEN-1);
		     break;
	     case ':' :
	     case '?' :
	     default :
		     return -1;
	     }
     }
     
     if (strlen(ctrl_host) == 0 || strlen(ctrl_service) == 0 || !isdef_cycletime)
          return -1;

     return 0;
}

void *receiver_thread_run(void *param)
{
	uint8_t data[MAX_PKT_SIZE];
	ssize_t data_len;
	
	while (true) {
		data_len = recv(sock, data, MAX_PKT_SIZE, 0);
		if (data_len == -1) {
			perror("Could not receive message");
		} else {
			uint64_t time;
			double u;
			if (!demarshaling_update(data, data_len, &time, &u)) {
				fprintf(stderr, "Demarshaling failed\n");
			} else {
				update_lock.lock();
				update.u = u;
				update_lock.unlock();
				update_ready.store(1);
			}
		}
	}
	
	return NULL;
}

int main(int argc, char *argv[])
{
	if (parse_cmdline_args(argc, argv) == -1) {
		usage(argv[0]);
		die(1);
	}

	// Create socket for communicating with controller.
	sock = datagram_client_socket(ctrl_host, ctrl_service);
	if (sock == -1) {
		perror("Could not create socket");
		die(1);
	}

	// Create thread receiving updates from controller.
	if (pthread_create(&thread, NULL, receiver_thread_run, NULL)) {
		perror("Could not create thread");
		die(1);
	}

	// Open log file if requested.
	FILE *log_file = NULL;
	if (strlen(log_file_path) > 0) {
		log_file = fopen(log_file_path, "w");
		if (log_file == NULL) {
			perror("Could not open log file");
			die(1);
		}
	}
	
	sf::RenderWindow window(sf::VideoMode(640, 480), "Inverted Pendulum");

	// Set initial conditions
	const double p_0 = 0;
	const double theta_0 = -5;
	Eigen::VectorXd x_0(4);
	x_0 << p_0, to_radians(theta_0), 0, 0;

	// Create a model with default parameters
	InvertedPendulum *ptr = new InvertedPendulum(x_0);

	// Load font
	sf::Font font;
	if (!font.loadFromFile("Roboto-Regular.ttf")) {
		std::cout << "Failed to load font!\n";
	}

	// Create text to display simulation time
	sf::Text text;
	text.setFont(font);
	text.setCharacterSize(24);
	const sf::Color grey = sf::Color(0x7E, 0x7E, 0x7E);
	text.setFillColor(grey);
	text.setPosition(480.0F, 360.0F);

	// Create a track for the cart
	sf::RectangleShape track(sf::Vector2f(640.0F, 2.0F));
	track.setOrigin(320.0F, 1.0F);
	track.setPosition(320.0F, 240.0F);
	const sf::Color light_grey = sf::Color(0xAA, 0xAA, 0xAA);
	track.setFillColor(light_grey);

	// Create the cart of the inverted pendulum
	sf::RectangleShape cart(sf::Vector2f(100.0F, 100.0F));
	cart.setOrigin(50.0F, 50.0F);
	cart.setPosition(320.0F, 240.0F);
	cart.setFillColor(sf::Color::Black);

	// Create the pole of the inverted pendulum
	sf::RectangleShape pole(sf::Vector2f(20.0F, 200.0F));
	pole.setOrigin(10.0F, 200.0F);
	pole.setPosition(320.0F, 240.0F);
	pole.setRotation(-theta_0);
	const sf::Color brown = sf::Color(0xCC, 0x99, 0x66);
	pole.setFillColor(brown);

	// Create a clock to run the simulation.
	// This clock is monotonically increasing.
	// Precision is the best that the operating system can provide.
	sf::Clock clock;

	// The system input.
	double u = 0;

	// First cycle starts now.
	uint64_t t_next_cycle_usec = clock.getElapsedTime().asMicroseconds();

	uint64_t t_next_log_output_usec = t_next_cycle_usec;

	while (window.isOpen()) {
		sf::Event event;
		while (window.pollEvent(event)) {
			switch (event.type) {
			case sf::Event::Closed:
				window.close();
				break;
			}
		}

		sf::Time t_current = clock.getElapsedTime();
		uint64_t t_current_usec = t_current.asMicroseconds();
		const std::string msg = std::to_string(t_current.asSeconds());
		text.setString("Time   " + msg.substr(0, msg.find('.') + 2));
		
		// If next cycle has started, sample plant state and send state to controller.
		if (t_next_cycle_usec <= t_current_usec) {
			double angle = ptr->GetState()(1);
			size_t data_len;
			uint8_t data[MAX_PKT_SIZE];
			data_len = marshaling_state(data, MAX_PKT_SIZE, t_current_usec, angle); 
			if (data_len == -1) {
				fprintf(stderr, "Could not marshal data.\n");
			} else if (send(sock, data, data_len, 0) == -1) {
				perror("Could not send update to controller");
			} else {
				//printf("State sent: time = %" PRIu64 " us  angle = %f degree\n", t_current_usec, angle);
			}
			
			t_next_cycle_usec += cycletime_usec;
		}
		
		// If an update from the controller is available, update system input.
		// If no update is available, keep the old value of the system input. 
		// If we can exchange a 1 by a 0, there was a new update ready.
		// Maybe, we have missed some updates, but we can always read the latest update.
		int expected_val = 1;
		int new_val = 0;
		if (update_ready.compare_exchange_weak(expected_val, new_val)) {
			update_lock.lock();
			u = update.u;
			update_lock.unlock();
			//printf("Update received: u = %f\n", u);
		}

		// Update the plant.
		ptr->Update(t_current.asSeconds(), u);
				
		// Update SFML drawings
		Eigen::VectorXd x = ptr->GetState();
		
		cart.setPosition(320.0F + 100 * x(0), 240.0F);
		pole.setPosition(320.0F + 100 * x(0), 240.0F);
		pole.setRotation(to_degrees(-x(1)));
		
		window.clear(sf::Color::White);
		window.draw(track);
		window.draw(cart);
		window.draw(pole);
		window.draw(text);
		window.display();

		if (log_file && t_next_log_output_usec <= t_current_usec) {
			double angle = ptr->GetState()(1);
			if (fprintf(log_file, "%" PRIu64 ",%f\n", t_current_usec, to_degrees(angle)) < 0)
				perror("Failed to write log entry");
			t_next_log_output_usec += LOG_INTERVAL_USEC;
		}
	}

	if (log_file)
		fclose(log_file);
	
	return 0;
}
