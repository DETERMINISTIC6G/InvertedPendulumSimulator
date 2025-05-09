cmake_minimum_required(VERSION 3.25)

#find_package(Boost 1.53 COMPONENTS program_options REQUIRED)
#include_directories(${Boost_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 17)

project(InvertedPendulumSimulator)

add_executable(simulate inverted_pendulum/inverted_pendulum.cc inverted_pendulum/inverted_pendulum.h apps/simulate.cc)

add_executable(simulate-pid inverted_pendulum/inverted_pendulum.cc inverted_pendulum/inverted_pendulum.h apps/simulate-pid.cc controller/pid.h controller/pid.cc)

find_package(SFML COMPONENTS graphics window system REQUIRED)
add_executable(visualization apps/visualization.cc inverted_pendulum/inverted_pendulum.h)
target_link_libraries(visualization sfml-graphics sfml-window sfml-system)
