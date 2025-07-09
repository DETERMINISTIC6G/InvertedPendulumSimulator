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

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "socket_utils.h"

ssize_t stream_server_sockets(const char *hostname, const char *service,
			      int addressfamily, int protocol,
			      int backlog, int *res_sockets, size_t max_nsockets)
{ 
     struct addrinfo *res, *addr;
     struct addrinfo hints;
     memset(&hints, 0, sizeof(hints));
     
     hints.ai_flags = AI_PASSIVE;
     hints.ai_socktype = SOCK_STREAM;
     hints.ai_family = addressfamily;
     hints.ai_protocol = protocol;

     int error = getaddrinfo(hostname, service, &hints, &res);
     if (error)
	  return -1;

     // Try to bind and listen on all returned addresses.
     size_t n = 0;
     for (addr = res; addr != NULL && n < max_nsockets; addr = addr->ai_next) {
	  int s;
	  s = socket(addr->ai_family, addr->ai_socktype,
		     addr->ai_protocol);
	  if (s == -1)
	       continue;

	  if (bind(s, addr->ai_addr, addr->ai_addrlen)) {
 	       close(s);
	       continue;
	  }

	  if (listen(s, backlog)) {
	       close(s);
	       continue;
	  }

	  // Socket successfully created, bound, and listening.
	  res_sockets[n++] = s;
     }

     freeaddrinfo(res);

     if (n == 0)
	  return -1;
     else
	  return n;
}

int stream_client_socket(const char *hostname, const char *service)
{
     int sock;
     struct addrinfo hints;
     struct addrinfo *res, *addr;

     memset(&hints, 0, sizeof(hints));
     hints.ai_flags = 0;
     hints.ai_socktype = SOCK_STREAM;
     // Try all address families (IPv4, IPv6, ...)
     hints.ai_family = AF_UNSPEC;
     // Try all protocols (IPPROTO_TCP, ...)
     hints.ai_protocol = 0;

     int error = getaddrinfo(hostname, service, &hints, &res);
     if (error)
	  return -1;

     // Try all proposed addresses until one works.
     int s = -1;
     for (addr = res; addr != NULL; addr = addr->ai_next) {
	  s = socket(addr->ai_family, addr->ai_socktype,
		     addr->ai_protocol);
	  if (s == -1)
	       continue;
	  
	  if (connect(s, addr->ai_addr, addr->ai_addrlen) == -1) {
	       close(s); 
	       continue;
	  }

	  // Socket created and connected.
	  break;
     }

     freeaddrinfo(res);

     return s;
}

ssize_t datagram_server_sockets(const char *hostname, const char *service,
				int addressfamily, int protocol,
				int *res_sockets, size_t max_nsockets)
{
     struct addrinfo *res, *addr;
     struct addrinfo hints;
     memset(&hints, 0, sizeof(hints));
     
     hints.ai_flags = AI_PASSIVE;
     hints.ai_socktype = SOCK_DGRAM;
     hints.ai_family = addressfamily;
     hints.ai_protocol = protocol;

     int error = getaddrinfo(hostname, service, &hints, &res);
     if (error)
	  return -1;

     // Try to bind socket gto all returned addresses.
     size_t n = 0;
     for (addr = res; addr != NULL && n < max_nsockets; addr = addr->ai_next) {
	  int s;
	  s = socket(addr->ai_family, addr->ai_socktype,
		     addr->ai_protocol);
	  if (s == -1)
	       continue;

	  if (bind(s, addr->ai_addr, addr->ai_addrlen)) {
 	       close(s);
	       continue;
	  }

	  // Socket successfully created, bound, and listening.
	  res_sockets[n++] = s;
     }

     freeaddrinfo(res);

     if (n == 0)
	  return -1;
     else
	  return n;
}

int datagram_client_socket(const char *hostname, const char *service)
{
     int sock;
     struct addrinfo hints;
     struct addrinfo *res, *addr;

     memset(&hints, 0, sizeof(hints));
     hints.ai_flags = 0;
     hints.ai_socktype = SOCK_DGRAM;
     // Try all address families (IPv4, IPv6, ...)
     hints.ai_family = AF_UNSPEC;
     // Try all protocols (IPPROTO_TCP, ...)
     hints.ai_protocol = 0;

     int error = getaddrinfo(hostname, service, &hints, &res);
     if (error)
	  return -1;

     // Try all proposed addresses until one works.
     int s = -1;
     for (addr = res; addr != NULL; addr = addr->ai_next) {
	  s = socket(addr->ai_family, addr->ai_socktype,
		     addr->ai_protocol);
	  if (s == -1)
	       continue;
	  
	  if (connect(s, addr->ai_addr, addr->ai_addrlen) == -1) {
	       close(s); 
	       continue;
	  }

	  // Socket created and connected.
	  break;
     }

     freeaddrinfo(res);

     return s;
}
