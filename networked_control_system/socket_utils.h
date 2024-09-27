/**
 * Copyright 2019 Frank Duerr (University of Stuttgart)
 *                frank.duerr@ipvs.uni-stuttgart.de
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY 
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND 
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
