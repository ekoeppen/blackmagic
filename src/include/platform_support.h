/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __PLATFORM_SUPPORT_H
#define __PLATFORM_SUPPORT_H

#ifndef __GENERAL_H
#	error "Include 'general.h' instead"
#endif

#if defined(LIBFTDI)
void platform_init(int argc, char **argv);
#else
void platform_init(void);
#endif

const char *platform_target_voltage(void);
int platform_hwversion(void);
void platform_timeout_set(uint32_t ms);
bool platform_timeout_is_expired(void);
void platform_delay(uint32_t delay);
void platform_srst_set_val(bool assert);
bool platform_target_get_power(void);
void platform_target_set_power(bool power);
void platform_request_boot(void);

#endif

