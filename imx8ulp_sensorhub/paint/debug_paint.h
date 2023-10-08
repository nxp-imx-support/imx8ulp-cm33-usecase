/*
 * Copyright 2021-2023, NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __DEBUG_H
#define __DEBUG_H

#include "stdio.h"

#define DEBUG_FLAG 1
#if DEBUG_FLAG
	#define Debug(__info,...) printf("Debug : " __info,##__VA_ARGS__) 
#endif

#endif

