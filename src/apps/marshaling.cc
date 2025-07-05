/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
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
		// TODO: only works on machines with same endianess
		memcpy(buffer, &n, 8);
	}
}

void demarshaling_double(const uint8_t buffer[8], double &n)
{
	if (is_big_endian()) {
		memcpy(&n, buffer, 8);
	} else {
		// TODO: only works on machines with same endianess
		memcpy(&n, buffer, 8);
	}
}

void marshaling_uint64(uint64_t n, uint8_t buffer[8])
{
	(*(uint64_t *) buffer) = htobe64(n);
}

void demarshaling_uint64(const uint8_t buffer[8], uint64_t &n)
{
	n = be64toh(*((uint64_t *) buffer));
}

ssize_t marshaling_state(uint8_t *data, size_t max_data_size, uint64_t time, double angle, double omega, double x, double v)
{
	size_t len = 0;
	
	if (max_data_size < sizeof(time) + sizeof(angle) + sizeof(omega) + sizeof(x) + sizeof(v))
		return -1;
	
	marshaling_uint64(time, data);
	len += sizeof(time);
	marshaling_double(angle, data+len);
	len += sizeof(angle);
	marshaling_double(omega, data+len);
	len += sizeof(omega);
	marshaling_double(x, data+len);
	len += sizeof(x);
	marshaling_double(v, data+len);
	len += sizeof(v);
	
	return len; 
}

bool demarshaling_state(const uint8_t *data, size_t data_size, uint64_t &time, double &angle, double &omega, double &x, double &v)
{
	size_t pos = 0;
	
	if (data_size < sizeof(time) + sizeof(angle) + sizeof(omega) + sizeof(x) + sizeof(v))
		return false;
	
	demarshaling_uint64(data, time);
	pos += sizeof(time);
	demarshaling_double(data+pos, angle);
	pos += sizeof(angle);
	demarshaling_double(data+pos, omega);
	pos += sizeof(omega);
	demarshaling_double(data+pos, x);
	pos += sizeof(x);
	demarshaling_double(data+pos, v);

	return true;
}

ssize_t marshaling_update(uint8_t *data, size_t max_data_size, uint64_t time, double u)
{
	size_t len = 0;
	
	if (max_data_size < sizeof(time) + sizeof(u))
		return -1;
	
	marshaling_uint64(time, data);
	len += sizeof(time);
	marshaling_double(u, data+len);
	len += sizeof(u);
	
	return len; 
}

bool demarshaling_update(const uint8_t *data, size_t data_size, uint64_t &time, double &u)
{
	size_t pos = 0;
	
	if (data_size < sizeof(time) + sizeof(u))
		return false;
	
	demarshaling_uint64(data, time);
	pos += sizeof(time);
	demarshaling_double(data+pos, u);

	return true;
}
