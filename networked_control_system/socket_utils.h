/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
 */

#ifndef SOCKET_UTILS_H
#define SOCKET_UTILS_H

#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>

/**
 * Create and bind one or several stream server sockets matching the given 
 * criteria and start listening for connection requests on each 
 * successfully created and bound socket. 
 * 
 * @param hostname hostname or IP address of network interface to bind
 * sockets. NULL for all available interface.
 * @param service service name or port number to bind sockets.
 * @param addressfamily requested address family (AF_INET, AF_INET6, ...)
 * or AF_UNSPEC for all address families.
 * @param protocol requested protocol (IPPROTO_TCP, ...) or 0 for all
 * protocols (matching the other given criteria).
 * @param backlog backlog size. 
 * @param res_sockets array receiving successfully created sockets. 
 * @param max_nsockets maximum number of entries in array sockets.
 * @return number of successfully created and bound sockets, i.e., number of 
 * entries in array sockets; -1 on error.
 */
ssize_t stream_server_sockets(const char *hostname, const char *service,
			      int addressfamily, int protocol, int backlog,
			      int *res_sockets, size_t max_nsockets);

/**
 * Create and connect a stream client socket to a given server.
 * This function will try all (suitable) protocols and addresses, until
 * a connection can be established or no option works.  
 * 
 * @param hostname hostname or IP address of server to connect to.
 * @param service service name or port number of server to connect to.
 * @return connected client socket; -1 on error.
 */
int stream_client_socket(const char *hostname, const char *service);

/**
 * Create and bind one or several datagram server sockets matching the given 
 * criteria. 
 * 
 * @param hostname hostname or IP address of network interface to bind
 * sockets. NULL for all available interface.
 * @param service service name or port number to bind sockets.
 * @param addressfamily requested address family (AF_INET, AF_INET6, ...)
 * or AF_UNSPEC for all address families.
 * @param protocol requested protocol (IPPROTO_UDP, ...) or 0 for all
 * protocols (matching the other given criteria).
 * @param res_sockets array receiving successfully created sockets. 
 * @param max_nsockets maximum number of entries in array sockets.
 * @return number of successfully created and bound sockets, i.e., number of 
 * entries in array sockets; -1 on error.
 */
ssize_t datagram_server_sockets(const char *hostname, const char *service,
				int addressfamily, int protocol,
				int *res_sockets, size_t max_nsockets);

/**
 * Create and "connect" a datagram client socket to a given server.
 * Connection in this context means that the destination address
 * is fixed such that the (same) destination address does not need to
 * be provided with every send operation (use send() instead of sendto()).
 * This function will try all (suitable) protocols and addresses, until
 * a "connection" can be established or no option works.
 * 
 * @param hostname hostname or IP address of server to connect to.
 * @param service service name or port number of server to connect to.
 * @return connected client socket; -1 on error.
 */
int datagram_client_socket(const char *hostname, const char *service);

#endif
