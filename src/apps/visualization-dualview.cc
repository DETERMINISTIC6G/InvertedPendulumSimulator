/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
 * SPDX-FileContributor: Elena Mostovaya (st169601@stud.uni-stuttgart.de)
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
#include <array>
#include <boost/tokenizer.hpp>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

#include "../inverted_pendulum/inverted_pendulum.h"

#define FRAME_RATE 30

#define MAX_STR_LEN 1024

char pathCSVFile1[MAX_STR_LEN];
char pathCSVFile2[MAX_STR_LEN];

const float visoffset = 350;

/**
 * Parse command line arguments as passed to main() and store them in
 * global variables.
 */
int parse_cmdline_args(int argc, char *argv[])
{
        int opt;

        memset(pathCSVFile1, 0, MAX_STR_LEN);
        memset(pathCSVFile2, 0, MAX_STR_LEN);

        while ((opt = getopt(argc, argv, "f:F:")) != -1) {
                switch (opt) {
                case 'f':
                        strncpy(pathCSVFile1, optarg, MAX_STR_LEN - 1);
                        break;
                case 'F':
                        strncpy(pathCSVFile2, optarg, MAX_STR_LEN - 1);
                        break;
                case ':':
                case '?':
                default:
                        return -1;
                }
        }

        if (strlen(pathCSVFile1) == 0 || strlen(pathCSVFile2) == 0)
                return -1;

        return 0;
}

bool read_states(const char *path, state_sequence_t &states_vis)
{
        std::ifstream infile(path);

        if (!infile.is_open()) {
                perror("Could not open states file");
                return -1;
        }

        std::string line;
        while (std::getline(infile, line)) {
                // Ignore empty lines with no characters
                if (line.empty())
                        continue;
                // Ignore lines starting with #
                if (line.rfind("#", 0) == 0)
                        continue;
                boost::char_separator<char> sep(",");
                boost::tokenizer<boost::char_separator<char>> tokens(line, sep);
                unsigned int token_cnt = 0;
                time_state_t time_state;
                for (const auto &token : tokens) {
                        double d = strtod(token.c_str(), NULL);
                        if (token_cnt == 0) {
                                time_state.first = d;
                        } else {
                                time_state.second[token_cnt - 1] = d;
                        }
                        token_cnt++;
                        if (token_cnt > 5) {
                                std::cerr << "Too many tokens in line: " << line << std::endl;
                                return false;
                        }
                }
                if (token_cnt < 5) {
                        std::cerr << "Too few tokens" << std::endl;
                        return false;
                }
                assert(token_cnt == 5);
                states_vis.push_back(time_state);
        }

        return true;
}

void prepare_states_vis(const state_sequence_t &states, state_sequence_t &states_vis)
{
        double period = 1.0 / FRAME_RATE;
        double t = 0.0;

        for (time_state_t time_state : states) {
                if (time_state.first > t) {
                        states_vis.push_back(time_state);
                        t += period;
                }
        }
}

double to_deg(float rad)
{
        return rad * (180.0 / M_PI);
}

int main(int argc, char *argv[])
{
        if (parse_cmdline_args(argc, argv) == -1) {
                // usage(argv[0]);
                exit(1);
        }

        state_sequence_t states1;
        state_sequence_t states2;
        state_sequence_t states_vis1;
        state_sequence_t states_vis2;

        if (!read_states(pathCSVFile1, states1)) {
                exit(1);
        }
        prepare_states_vis(states1, states_vis1);
        if (!read_states(pathCSVFile2, states2)) {
                exit(1);
        }
        prepare_states_vis(states2, states_vis2);

        sf::RenderWindow window(sf::VideoMode(1024, 480), "Inverted Pendulum");

        // Load font
        sf::Font font;
        if (!font.loadFromFile("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf")) {
                std::cerr << "Failed to load font!\n";
        }

        // Create text to display heading
        sf::Text text1;
        text1.setFont(font);
        text1.setCharacterSize(24);
        const sf::Color grey = sf::Color(0x7E, 0x7E, 0x7E);
        text1.setFillColor(grey);
        text1.setPosition(480.0F, 0.0F);
        text1.setString("Wireline");

        sf::Text text2;
        text2.setFont(font);
        text2.setCharacterSize(24);
        text2.setFillColor(grey);
        text2.setPosition(480.0F, 0.0F + visoffset);
        text2.setString("Wireless");

        // Create a track for the cart
        sf::RectangleShape track1(sf::Vector2f(1024.0F, 2.0F));
        track1.setOrigin(512.0F, 1.0F);
        track1.setPosition(512.0F, 240.0F);
        const sf::Color light_grey = sf::Color(0xAA, 0xAA, 0xAA);
        track1.setFillColor(light_grey);

        sf::RectangleShape track2(sf::Vector2f(1024.0F, 2.0F));
        track2.setOrigin(512.0F, 1.0F);
        track2.setPosition(512.0F, 240.0F + visoffset);
        track2.setFillColor(light_grey);

        // Create the cart of the inverted pendulum
        sf::RectangleShape cart1(sf::Vector2f(100.0F, 100.0F));
        cart1.setOrigin(50.0F, 50.0F);
        cart1.setPosition(320.0F, 240.0F);
        cart1.setFillColor(sf::Color::Black);

        sf::RectangleShape cart2(sf::Vector2f(100.0F, 100.0F));
        cart2.setOrigin(50.0F, 50.0F);
        cart2.setPosition(320.0F, 240.0F + visoffset);
        cart2.setFillColor(sf::Color::Black);

        // Create the pole of the inverted pendulum
        sf::RectangleShape pole1(sf::Vector2f(20.0F, 200.0F));
        pole1.setOrigin(10.0F, 200.0F);
        const sf::Color brown = sf::Color(0xCC, 0x99, 0x66);
        pole1.setFillColor(brown);

        sf::RectangleShape pole2(sf::Vector2f(20.0F, 200.0F));
        pole2.setOrigin(10.0F, 200.0F);
        pole2.setFillColor(brown);

        // Create a clock to run the simulation
        sf::Clock clock;

        state_sequence_t::iterator it1 = states_vis1.begin();
        state_sequence_t::iterator it2 = states_vis2.begin();

        while (window.isOpen() && it1 != states_vis1.end() && it2 != states_vis2.end()) {
                sf::Event event;
                while (window.pollEvent(event)) {
                        switch (event.type) {
                        case sf::Event::Closed:
                                window.close();
                                break;
                        }
                }

                time_state_t time_state1 = *it1;
                time_state_t time_state2 = *it2;
                double tnext;
                if (time_state1.first == time_state2.first) {
                        it1++;
                        it2++;
                        tnext = time_state1.first;
                } else if (time_state1.first < time_state2.first) {
                        it1++;
                        tnext = time_state1.first;
                } else {
                        it2++;
                        tnext = time_state2.first;
                }

                // Wait until next frame is due.
                sf::Time time;
                uint64_t time_us;
                uint64_t frame_time_us;
                do {
                        time = clock.getElapsedTime();
                        time_us = time.asMicroseconds();
                        frame_time_us = (uint64_t)(1000000.0 * tnext);
                } while (time_us < frame_time_us);

                // Update the simulation

                float cart_x1 = time_state1.second[0];
                float pole_angle_deg1 = to_deg(time_state1.second[2]);

                float cart_x2 = time_state2.second[0];
                float pole_angle_deg2 = to_deg(time_state2.second[2]);

                // Update SFML drawings
                cart1.setPosition(320.0 + 100 * cart_x1, 240.0);
                pole1.setPosition(320.0 + 100 * cart_x1, 240.0);
                pole1.setRotation(-pole_angle_deg1);

                cart2.setPosition(320.0 + 100 * cart_x2, 240.0 + visoffset);
                pole2.setPosition(320.0 + 100 * cart_x2, 240.0 + visoffset);
                pole2.setRotation(-pole_angle_deg2);

                window.clear(sf::Color::White);
                window.draw(track1);
                window.draw(cart1);
                window.draw(pole1);
                window.draw(track2);
                window.draw(cart2);
                window.draw(pole2);
                window.draw(text1);
                window.draw(text2);
                window.display();
        }

        return 0;
}
