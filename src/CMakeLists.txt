cmake_minimum_required(VERSION 3.25)

#find_package(Boost 1.53 COMPONENTS program_options REQUIRED)
#include_directories(${Boost_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 17)

project(InvertedPendulumSimulator)

add_executable(simulate inverted_pendulum/inverted_pendulum.cc inverted_pendulum/inverted_pendulum.h apps/simulate.cc)

add_executable(simulate-pid inverted_pendulum/inverted_pendulum.cc inverted_pendulum/inverted_pendulum.h apps/simulate-pid.cc controller/pid.h controller/pid.cc)

add_executable(simulate-position_angle inverted_pendulum/inverted_pendulum.cc inverted_pendulum/inverted_pendulum.h apps/simulate-position_angle.cc controller/pid.h controller/pid.cc)

add_executable(simulate-lqr inverted_pendulum/inverted_pendulum.cc inverted_pendulum/inverted_pendulum.h apps/simulate-lqr.cc controller/lqr.h controller/lqr.cc)

add_executable(simulate-lqr-position_angle inverted_pendulum/inverted_pendulum.cc inverted_pendulum/inverted_pendulum.h apps/simulate-lqr-position_angle.cc controller/lqr.h controller/lqr.cc)

find_package(SFML COMPONENTS graphics window system REQUIRED)
add_executable(visualization apps/visualization.cc inverted_pendulum/inverted_pendulum.h)
target_link_libraries(visualization sfml-graphics sfml-window sfml-system)

add_executable(simulate-event_queue inverted_pendulum/inverted_pendulum.cc inverted_pendulum/inverted_pendulum.h 
                                    apps/simulate-event_queue.cc 
                                    controller/pid.h controller/pid.cc
                                    controller/lqr.h controller/lqr.cc
                                    events/event_queue.h events/event_queue.cc
                                    )

