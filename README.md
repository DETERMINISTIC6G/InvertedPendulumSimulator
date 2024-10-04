This repository is a fork of the [InvertedPendulum simulator](https://github.com/jasleon/Inverted-Pendulum) developed by Antonio Sanchez.

This implementation was extended in the context of the [DETERMINISTIC6G](https://deterministic6g.eu/) research project to support the evaluation of networked control systems communicating over Time-Sensitive Networks (TSN) including, in particular, wireless 5G/6G TSN bridges.
Typically, such wireless TSN bridges have fundamentally different characteristics with respect to port-to-port (bridge) delay (stochastic heavy-tailed delay, orders of magnitude higher than for wired TSN bridges). 
Since delay is critical for control systems, we want to evaluate the Quality of Control (QoC) if the networked control system is exposed to this characteristic network delay.

To enable such evaluations, we modified the original implementation of the inverted pendulum as follows:

* We separate the inverted pendulum (plant) from the controller (hosted for instance in an edge cloud environment) and connect them through a UDP packet stream to close the control loop over the network. 
* We then either use network emulation or simulation to emulate or simulate the characteristic network delay induced by wireless TSN bridges, respectively. 

For network emulation, we use the [Network Delay Emulator](https://github.com/DETERMINISTIC6G/NetworkDelayEmulator) developed by the DETERMINISTIC6G project.

For network simulation, we use the [DETERMINISTIC6G extensions](https://github.com/DETERMINISTIC6G/deterministic6g) of the OMNeT++/INET network simulator. 

Both, simulations and emulations, utilize the [delay measurements](https://github.com/DETERMINISTIC6G/deterministic6g_data) from real 5G networks of the DETERMINISTIC6G project.  

In the following, we describe how to use the distributed inverted pendulum together with network emulation or simulation.
We focus this description on the extensions made to the original inverted pendulum implementation (without distributed implementation).
If you are interested in the original, non-distributed pendulum, you can find the original README file in a [separate file](), or in the [original repository](https://github.com/jasleon/Inverted-Pendulum).

# Network Emulation & Inverted Pendulum

t.b.d.

# Network Simulation & Inverted Pendulum

t.b.d.

# Acknowledgements

t.b.d.
