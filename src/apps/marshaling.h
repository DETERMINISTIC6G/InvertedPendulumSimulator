/**
 * SPDX-FileCopyrightText: 2025 University of Stuttgart
 * 
 * SPDX-License-Identifier: MIT
 * 
 * SPDX-FileContributor: Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de)
 */

#ifndef MARSHALING_H
#define MARSHALING_H

#include <sys/types.h>
#include <stdint.h>

ssize_t marshaling_state(uint8_t *data, size_t max_data_size, uint64_t time, double angle, double omega, double x, double v); 

bool demarshaling_state(const uint8_t *data, size_t data_size, uint64_t &time, double &angle, double &omega, double &x, double &v);

ssize_t marshaling_update(uint8_t *data, size_t max_data_size, uint64_t time, double u); 

bool demarshaling_update(const uint8_t *data, size_t data_size, uint64_t &time, double &u);

#endif
