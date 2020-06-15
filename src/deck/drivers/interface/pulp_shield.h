/*----------------------------------------------------------------------------*
 * Copyright (C) 2018-2019 ETH Zurich, Switzerland                            *
 * All rights reserved.                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 * See LICENSE.apache.md in the top directory for details.                    *
 * You may obtain a copy of the License at                                    *
 *                                                                            *
 *     http://www.apache.org/licenses/LICENSE-2.0                             *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 *                                                                            *
 * File:    pulp_shield.h                                                     *
 * Author:  Daniele Palossi <dpalossi@iis.ee.ethz.ch>                         *
 * Date:    04.09.2019                                                        *
 *----------------------------------------------------------------------------*/

#ifndef _PULPSHIELD_H_
#define _PULPSHIELD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "stabilizer_types.h"
#include "deck_core.h"
#include "deck.h"
#include "arm_math.h"
#include "log.h"
#include "param.h"
#include "debug.h"

#define PULP_SHIELD
#define WATCH_DOG_EN

#define PULP_TARGET_H		1.50f
#define SLEEP_THRESHOLD		20000  // would sleep seconds
// #define SLEEP_THRESHOLD		2000  // would sleep seconds
#define ERROR_THRESHOLD		750

#define PULP_TAKEOFF_RATE	10
#define PULP_NAVIGATE_RATE	100
#define PULP_LANDING_RATE	10

#define PULP_STEPS_TAKEOFF	200
#define PULP_STEPS_HOVERING	200
#define PULP_STEPS_LANDING	200

/* For conversion from fixed-point to float */
#define FIXEDPT_FBITS		11 /* Q5.11 */
#define FIXED_BITVALUE 		(1.0f / (1<<FIXEDPT_FBITS))


void PULPShieldOn();

void PULPShieldOff();

void GPIOInit();

void GPIOOn();

void GPIOOff();

void takeOff();

void getSetpoint();

void resetSetpoint();

float fixed2float(int16_t *x);

uint32_t getLastPULPUpdate();

setpoint_t PULPShieldGetSetpoint() ;


#endif /* _PULPSHIELD_H_ */
