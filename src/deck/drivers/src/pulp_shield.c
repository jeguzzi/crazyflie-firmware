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
 * File:    pulp_shield.c                                                     *
 * Author:  Daniele Palossi <dpalossi@iis.ee.ethz.ch>                         *
 * Date:    04.09.2019                                                        *
 *----------------------------------------------------------------------------*/


#include "pulp_shield.h"
#include "system.h"

static float x_;
static float y_;
static float z_;
static float phi_;

static uint32_t step, lastUpdate, sleep;
static bool isInit          = false;
static bool isEnabled         = false;


// setpoint_t PULPShieldGetSetpoint() {
//   return setpoint_;
// }



static bool PULPShieldTest() {

  DEBUG_PRINT("First PULP-Shield test passed!\n");
  // GPIO/SPI test should go here
  return true;
}


void PULPShieldOn() {

  x_      = 0.0f;
  y_      = 0.0f;
  z_      = 0.0f;
  phi_    = 0.0f;
  lastUpdate   = xTaskGetTickCount();

  // Set GPIO on (power on the shield)
  GPIOOn();
  // Enable SPI
  spiBeginSlave();

  // Reset task's step counter
  step = 1;

  sleep = xTaskGetTickCount();

  // Enable must be set last
  isEnabled = true;
}


void PULPShieldOff() {

  // Enable must be set first
  isEnabled = false;

  // Set GPIO low
  GPIOOff();

  // // Reset internal state
  // resetSetpoint();

  // Reset task's step counter
  step = 1;
}


void GPIOInit() {

  //Enable clock for GPIOC
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  //Initialize struct
  GPIO_InitTypeDef GPIO_InitStruct;
  //Pin 12
  GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_12;
  //Mode output
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
  //Output type push-pull
  GPIO_InitStruct.GPIO_OType  = GPIO_OType_PP;
  //Without pull resistors
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  //50MHz pin speed
  GPIO_InitStruct.GPIO_Speed  = GPIO_Speed_50MHz;
  //Initialize pins on GPIOG port
  GPIO_Init(GPIOC, &GPIO_InitStruct);
}


void GPIOOn() {

  //Set PC12 pin HIGH
  GPIO_SetBits(GPIOC, GPIO_Pin_12);
}


void GPIOOff(){

  //Set PC12 pin LOW
  GPIO_ResetBits(GPIOC, GPIO_Pin_12);
}


void getPULPOutputs() {

  uint8_t data_rx[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t data_tx[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  spiBeginTransactionSlave(SPI_BAUDRATE_2MHZ);
  spiExchangeSlave(8, data_tx, data_rx);
  spiEndTransactionSlave();

  x_     = fixed2float((int16_t*)&data_rx[0]);
  y_    = fixed2float((int16_t*)&data_rx[2]);
  z_    = fixed2float((int16_t*)&data_rx[4]);
  phi_  = fixed2float((int16_t*)&data_rx[6]);

  DEBUG_PRINT("FRONTNET: %f %f %f %f\n", (double)x_, (double)y_, (double)z_, (double)phi_);
}

float fixed2float(int16_t *x) {
  if((~x[0]) < x[0])
    return -FIXED_BITVALUE * ((~x[0])+1);
  else
    return FIXED_BITVALUE * x[0];
}


uint32_t getLastPULPUpdate() {
  return lastUpdate;
}


static void PULPShieldTask(void *param) {

  DEBUG_PRINT("PULPShieldTask Begin\n");

  systemWaitStart();

  while(1) {

      uint32_t currentTime = xTaskGetTickCount();
    /**** INITIAL TIMER ****/
      if(currentTime-sleep<SLEEP_THRESHOLD) {
        lastUpdate = xTaskGetTickCount();
        vTaskDelay(200);
      }
      else {
      if(isEnabled) {
        if((currentTime-lastUpdate) > PULP_NAVIGATE_RATE) {
          getPULPOutputs();
          lastUpdate = xTaskGetTickCount();
          step++;
        }
      } else {
        vTaskDelay(200);
      }
      }
  } // close while

  DEBUG_PRINT("PULPShieldTask Done\n");
}

static void PULPShieldInit() {

  if(isInit) return;

  xTaskCreate(PULPShieldTask,        // pointer to the task entry function
        PULP_SHIELD_TASK_NAME,    // name for the task
        PULP_SHIELD_TASK_STACKSIZE,  // number of words for task's stack
        NULL,            // the task's parameter
        PULP_SHIELD_TASK_PRI,    // task priority
        NULL);            // pass a handle

  GPIOInit();
  // GPIOOff();
  PULPShieldOn();

  isInit = true;

  DEBUG_PRINT("PULP-Shield Initialized\n");
}

static const DeckDriver PULPShieldDriver = {
    .name = "PULPShield",
    .init = PULPShieldInit,
    .test = PULPShieldTest,
};


DECK_DRIVER(PULPShieldDriver);


LOG_GROUP_START(frontnet)
LOG_ADD(LOG_FLOAT, x, &x_)
LOG_ADD(LOG_FLOAT, y, &y_)
LOG_ADD(LOG_FLOAT, z, &z_)
LOG_ADD(LOG_FLOAT, phi, &phi_)
LOG_ADD(LOG_UINT32, lastUpdate, &lastUpdate)
LOG_GROUP_STOP(frontnet)
