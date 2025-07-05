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

#include "marshaling.h"

// Non-standard functions but available on Linux.
#include <endian.h>

#include <string.h>

bool is_big_endian()
{
	return (42ll == htobe64(42ll));
}

void marshaling_double(double n, uint8_t buffer[8])
{
	if (is_big_endian()) {
		memcpy(buffer, &n, 8);
	} else {
		// TODO: not correct (only works on machines with same endianess)
		memcpy(buffer, &n, 8);
	}
}

void demarshaling_double(const uint8_t buffer[8], double *n)
{
	if (is_big_endian()) {
		memcpy(n, buffer, 8);
	} else {
		// TODO: not correct (only works on machines with same endianess)
		memcpy(n, buffer, 8);
	}
}

void marshaling_uint64(uint64_t n, uint8_t buffer[8])
{
	(*(uint64_t *) buffer) = htobe64(n);
}

void demarshaling_uint64(const uint8_t buffer[8], uint64_t *n)
{
	*n = be64toh(*((uint64_t *) buffer));
}

ssize_t marshaling_state(uint8_t *data, size_t max_data_size, uint64_t time, double angle)
{
	size_t len = 0;
	
	if (max_data_size < sizeof(time) + sizeof(angle))
		return -1;
	
	marshaling_uint64(time, data);
	len += sizeof(time);
	marshaling_double(angle, data+sizeof(time));
	len += sizeof(angle);
	
	return len; 
}

bool demarshaling_state(const uint8_t *data, size_t data_size, uint64_t *time, double *angle)
{
	size_t pos = 0;
	
	if (data_size < sizeof(*time) + sizeof(*angle))
		return false;
	
	demarshaling_uint64(data, time);
	pos += sizeof(*time);
	demarshaling_double(data+sizeof(time), angle);

	return true;
}

ssize_t marshaling_update(uint8_t *data, size_t max_data_size, uint64_t time, double u)
{
	size_t len = 0;
	
	if (max_data_size < sizeof(time) + sizeof(u))
		return -1;
	
	marshaling_uint64(time, data);
	len += sizeof(time);
	marshaling_double(u, data+sizeof(time));
	len += sizeof(u);
	
	return len; 
}

bool demarshaling_update(const uint8_t *data, size_t data_size, uint64_t *time, double *u)
{
	size_t pos = 0;
	
	if (data_size < sizeof(*time) + sizeof(*u))
		return false;
	
	demarshaling_uint64(data, time);
	pos += sizeof(*time);
	demarshaling_double(data+sizeof(time), u);

	return true;
}
