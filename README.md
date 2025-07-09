A simulator of an inverted pendulum for evaluating the performance of networked control systems whose communication is subject to packet delay. 

This simulator was implemented as part of the [DETERMINISTIC6G](https://deterministic6g.eu/) research project to support the evaluation of networked control systems communicating over Time-Sensitive Networks (TSN) including wireless 5G/6G TSN bridges.
Typically, such wireless TSN bridges have fundamentally different characteristics with respect to port-to-port (bridge) delay (stochastic heavy-tailed delay distribution, orders of magnitude higher than for wired TSN bridges). 
Since delay is critical for control systems, we want to evaluate the Quality of Control (QoC) if the networked control system is exposed to this characteristic network delay.

# Background

An inverted pendulum is a textbook example of a control system. As depicted in the figure below, it consists of an inverted pendulum mounted on a cart.
The goal is to keep the pendulum in an upright position, and optionally to also control the position of the cart with respect to a reference position.
Obviously, in contrast to an ordinary non-inverted pendulum, the inverted pendulum is an open-loop unstable system since the pendulum will tip over without control input.
To prevent the pendulum from tipping over, the cart can be accelerated by a force onto the cart.
This acceleration will create a torque onto the pendulum, accelerating the pendulum around its axis.

```
                       /
                      / inverted pendulum at angle Theta
                     /
                ----O----
               |  cart   |----> force F
                -O-----O-
---------------------------------------------------------->
track                                                    x
```

We are interested in controlling the pendulum over a communication network from a remote controller, i.e., the state of the pendulum is sampled at the mobile device (pendulum), sent over a communication network to the remote controller calculating the control input (force F), which is then sent back to the mobile device and applied to the cart.
Both, samples and control input exchanged between plant and controller, might suffer a packet delay depending on the characteristic packet delay of the communication network.
On the one hand. our pendulum simulator supports the integration with a real communication network or emulated network, e.g., the [network delay emulator](https://github.com/DETERMINISTIC6G/NetworkDelayEmulator) also developed by the DETERMINISTIC6G project.
On the other hand, the delay of packets can be pre-computed, e.g., using the [DETERMINISTIC6G simulation framework](https://github.com/DETERMINISTIC6G/deterministic6g), and then fed into the physical pendulum simulation through a file.

Evaluations and results produced with this simulation framework, we refer to [Deliverable D4.5](https://deterministic6g.eu/index.php/library-m/deliverables) of the DETERMINISTIC6G project. 

# Overview of the Code

The code is structured as follows:

* Directory `src/app`: various applications for simulating the pendulum (angle or angle & postion) and visualizing the pendulum movement in real-time. Best place to start to figure out how to use the implementation. The different applications are described further below. 
* Directory `src/controller`: implementations of different controllers (PID and LQR).
* Directory `src/inverted_pendulum`: implementation of the physics simulation of the pendulum.
* Further directories in folder `src`: various helper functions
* Directory `scripts`: different script, e.g., to calculate the optimal gain matrix for LQR with Mathematica.
* Directory `jupyter`: Jupyter notebooks for the analysis of results. 

In more detail, the following applications are included in folder `src/app`:

* `simulate-lqr`: showcase how to use pendulum and LQR to control angle (no network delay).
* `simulate-pid`: showcase how to use pendulum and PID controller to control angle (no network delay).
* `simulate-lqr-position_angle`: showcase how to use pendulum and LQR to control position and angle (no network delay).
* `simulate-position_angle`: showcase how to use pendulum and PID controller to control position and angle (no network delay).
* `simulate-event_queue`: showcase how to use the control system simulation to control angle.
The physical system simulation expects, as input, a packet trace from a network simulation, which simulates characteristic 5G network delays between the plant and the controller.
* `simulate-agv`: showcase how to use the control system simulation to control position and angle, where the position varies over time according to a predefined trajectory x(t) (i.e., the AGV moves intentionally). The physical system simulation expects, as input, a packet trace from a network simulation, which simulates characteristic 5G network delays between the AGV and the controller.
* `ncs-plant` / `ncs-controller`: networked control system with real network or emulated network (plant and controller communicating via sockets). Can be used together with [DETERMINISTIC6G network delay emulator](https://github.com/DETERMINISTIC6G/NetworkDelayEmulator) to emulate characteristic network delay between plant and controller.
* `visualization`: visualization of recorded pendulum state (animation of pendulum)
* `visualization-dualview`: visualization of recorded pendulum state (animation of pendulum), showing two pendulums simultaneously for visual comparison.

# Building the Apps

## Prerequisites

The visualization apps require the SFML library, which, for instance, can be installed on Debian systems as follows:

```(console)
$ sudo apt install libsfml-dev
```
Moreover, the boost odeint library is required :

```(console)
$ sudo apt install libboost-dev
```

## Building

The typical cmake process:

```(console)
$ cd src
$ mkdir build
$ cd build
$ cmake ..
$ make
``` 

# File Formats

## Packet Trace

The simulator apps `simulate-event_queue` and `simulate-agv` read a packet trace from a file, including the timestamps of packets (sensor samples) sent by the plant, and time of control input (u = force onto cart) arriving at the plant.

The packet trace can be generated using OMNeT++/INET with DETERMINISTIC6G extensions. 

The file format is CSV, with the following columns:

```
# pctNumber,rcvdTime,sendTime
```
* pctNumber: sequence number of the packet
* rcvdTime:  timestamp of control response arrival at the plant [s]
* sendTime:  timestamp  of state information sent from the plant [s]

## State Trace

The simulation apps write a trace of the state of the pendulum to a file, which can be used for visualization or analysis.

The file format is CSV, with the following columns:

```
# t,x,v,phi,omega
```

* t: timestamp of state [s]
* x: position of cart [m]
* v: velocity of cart [m/s]
* phi: angle of pole [rad]
* omega: angular velocity of pole [rad/s]

# Acknowledgements

The extensions in this repository for networked control systems have been made in the context of the DETERMINISTIC6G project, which has received funding from the European Union's Horizon Europe research and innovation programme under grant agreement No. 101096504.

[DETERMINISTIC6G Project Website](https://deterministic6g.eu/).

DETERMINISTIC6G e-mail: coordinator@deterministic6g.eu

The visualization of the pendulum is based on code originally developed by [Antonio Sanchez](https://github.com/jasleon/Inverted-Pendulum).
